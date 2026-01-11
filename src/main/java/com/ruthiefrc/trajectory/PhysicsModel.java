package com.ruthiefrc.trajectory;

/**
 * Physics model for projectile motion including gravity, drag, and Magnus effect.
 */
public class PhysicsModel {
    private final CalibrationParameters params;
    
    public PhysicsModel() {
        this(new CalibrationParameters());
    }
    
    public PhysicsModel(CalibrationParameters params) {
        this.params = params;
    }
    
    /**
     * Compute the derivative of the state (accelerations).
     * Implements Newton's Second Law with gravity, drag, and Magnus forces.
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
                * PhysicsConstants.BALL_CROSS_SECTION * speedSquared / PhysicsConstants.BALL_MASS_KG;
            Vector3D dragDirection = velocity.scale(-1.0 / speed);
            dragAccel = dragDirection.scale(dragMagnitude);
        } else {
            dragAccel = new Vector3D(0, 0, 0);
        }
        
        // Magnus force: F_magnus = Cm * (ω × v)
        // This creates lift perpendicular to both velocity and spin
        Vector3D magnusForce = spin.cross(velocity).scale(params.magnusCoefficient);
        Vector3D magnusAccel = magnusForce.scale(1.0 / PhysicsConstants.BALL_MASS_KG);
        
        // Total acceleration
        Vector3D totalAccel = gravityAccel.add(dragAccel).add(magnusAccel);
        
        // Spin decay (simple model - spin decays due to air resistance)
        Vector3D spinDerivative = spin.scale(-0.01); // small decay factor
        
        return new StateDerivative(velocity, totalAccel, spinDerivative);
    }
    
    /**
     * Apply a collision impulse to the state.
     * Used for bounces off HUB surfaces.
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
        
        // New velocity
        Vector3D newVelocity = newVNormal.add(newVTangent);
        
        // Update spin (simplified - spin can change due to friction)
        Vector3D newSpin = state.spin.scale(0.95);
        
        return new ProjectileState(state.position, newVelocity, newSpin, state.time);
    }
    
    public CalibrationParameters getParameters() {
        return params;
    }
}
