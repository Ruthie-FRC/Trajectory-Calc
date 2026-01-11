package com.ruthiefrc.trajectory;

/**
 * Physics model for projectile motion including gravity, drag, and Magnus effect.
 * Supports runtime-configurable projectile properties.
 */
public class PhysicsModel {
    private final CalibrationParameters params;
    private final ProjectileProperties projectile;
    
    public PhysicsModel() {
        this(new CalibrationParameters(), new ProjectileProperties());
    }
    
    public PhysicsModel(CalibrationParameters params) {
        this(params, new ProjectileProperties());
    }
    
    public PhysicsModel(CalibrationParameters params, ProjectileProperties projectile) {
        this.params = params;
        this.projectile = projectile;
    }
    
    /**
     * Compute the derivative of the state (accelerations).
     * Implements Newton's Second Law with gravity, drag, and Magnus forces.
     * Enhanced spin modeling for foam balls.
     */
    public StateDerivative computeDerivative(ProjectileState state) {
        Vector3D position = state.position;
        Vector3D velocity = state.velocity;
        Vector3D spin = state.spin;
        
        // Gravity force (always downward in -z)
        Vector3D gravityAccel = new Vector3D(0, 0, -PhysicsConstants.GRAVITY);
        
        // Drag force: F_drag = -0.5 * ρ * Cd * A * |v|² * (v/|v|)
        double speedSquared = velocity.magnitudeSquared();
        Vector3D dragAccel;
        if (speedSquared > 1e-6) {
            double speed = Math.sqrt(speedSquared);
            double dragMagnitude = 0.5 * PhysicsConstants.AIR_DENSITY * params.dragCoefficient
                * projectile.crossSectionalArea * speedSquared / projectile.massKg;
            Vector3D dragDirection = velocity.scale(-1.0 / speed);
            dragAccel = dragDirection.scale(dragMagnitude);
        } else {
            dragAccel = new Vector3D(0, 0, 0);
        }
        
        // Magnus force: F_magnus = Cm * (ω × v)
        // Enhanced: Magnitude scales with both spin rate and velocity
        // This creates lift perpendicular to both velocity and spin
        double spinMagnitude = spin.magnitude();
        double velocityMagnitude = Math.sqrt(speedSquared);
        Vector3D magnusForce;
        
        if (spinMagnitude > 1e-6 && velocityMagnitude > 1e-6) {
            // Magnus force proportional to spin × velocity for foam balls
            // Coefficient includes ball radius effect
            Vector3D magnusDirection = spin.cross(velocity);
            double magnusScaling = params.magnusCoefficient * spinMagnitude * velocityMagnitude;
            magnusForce = magnusDirection.scale(magnusScaling);
        } else {
            magnusForce = new Vector3D(0, 0, 0);
        }
        Vector3D magnusAccel = magnusForce.scale(1.0 / projectile.massKg);
        
        // Total acceleration
        Vector3D totalAccel = gravityAccel.add(dragAccel).add(magnusAccel);
        
        // Spin decay for foam balls
        // Decay rate increases with velocity (air resistance on spinning ball)
        double spinDecayFactor = params.spinDecayRate * (1.0 + velocityMagnitude / 20.0);
        Vector3D spinDerivative = spin.scale(-spinDecayFactor);
        
        return new StateDerivative(velocity, totalAccel, spinDerivative);
    }
    
    /**
     * Apply a collision impulse to the state.
     * Used for bounces off HUB surfaces.
     * Enhanced with spin effects on post-contact trajectory.
     */
    public ProjectileState applyCollision(ProjectileState state, Vector3D surfaceNormal) {
        // Decompose velocity into normal and tangential components
        double vNormal = state.velocity.dot(surfaceNormal);
        Vector3D vNormalVec = surfaceNormal.scale(vNormal);
        Vector3D vTangent = state.velocity.subtract(vNormalVec);
        
        // Apply restitution to normal component (bounce)
        Vector3D newVNormal = vNormalVec.scale(-params.restitutionCoefficient);
        
        // Apply friction to tangential component
        Vector3D newVTangent = vTangent.scale(1.0 - params.frictionCoefficient);
        
        // Spin-induced modification to tangential velocity
        // Magnus effect at contact: spin can add or reduce tangential velocity
        double spinMagnitude = state.spin.magnitude();
        if (spinMagnitude > 1e-6) {
            // Find spin component perpendicular to normal (spin affecting tangential motion)
            Vector3D spinTangent = state.spin.subtract(surfaceNormal.scale(state.spin.dot(surfaceNormal)));
            double spinEffect = spinTangent.magnitude() * PhysicsConstants.SPIN_FRICTION_COUPLING;
            
            // Direction of spin effect on tangential velocity
            if (spinTangent.magnitude() > 1e-6) {
                Vector3D spinDirection = spinTangent.scale(1.0 / spinTangent.magnitude());
                Vector3D spinVelocityContribution = spinDirection.cross(surfaceNormal).scale(spinEffect);
                newVTangent = newVTangent.add(spinVelocityContribution);
            }
        }
        
        // New velocity
        Vector3D newVelocity = newVNormal.add(newVTangent);
        
        // Update spin: loses energy on contact (foam compression) and friction
        // Spin reduction depends on impact severity and surface friction
        double impactSeverity = Math.abs(vNormal) / 10.0; // normalized by typical velocity
        double spinReduction = PhysicsConstants.SPIN_BOUNCE_REDUCTION_FACTOR * (1.0 + impactSeverity * 0.3);
        spinReduction = Math.min(0.95, spinReduction); // Cap at 95% reduction
        Vector3D newSpin = state.spin.scale(1.0 - spinReduction);
        
        // Additional spin change from friction torque
        // Friction can induce or modify spin based on tangential velocity mismatch
        if (vTangent.magnitude() > 0.1) {
            Vector3D frictionTorque = vTangent.cross(surfaceNormal).scale(params.frictionCoefficient * 0.1);
            newSpin = newSpin.add(frictionTorque);
        }
        
        return new ProjectileState(state.position, newVelocity, newSpin, state.time);
    }
    
    public CalibrationParameters getParameters() {
        return params;
    }
    
    public ProjectileProperties getProjectile() {
        return projectile;
    }
}
