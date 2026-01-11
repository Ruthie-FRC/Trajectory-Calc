package frc.robot.trajectory.solver;

import frc.robot.trajectory.physics.PhysicsConstants;
import frc.robot.trajectory.physics.PhysicsModel;
import frc.robot.trajectory.physics.ProjectileState;
import frc.robot.trajectory.physics.StateDerivative;
import frc.robot.trajectory.util.Vector3D;

/**
 * Fourth-order Runge-Kutta (RK4) numerical integrator.
 * Provides accurate integration of the projectile's equations of motion.
 * Supports adaptive timestep for improved precision near collisions.
 */
public class RK4Integrator {
    protected final PhysicsModel physicsModel;
    protected final double timestep;
    private final boolean adaptiveTimestep;
    private final double fineTimestep;
    private final double collisionProximityThreshold;
    
    public RK4Integrator(PhysicsModel physicsModel, double timestep) {
        this(physicsModel, timestep, false, PhysicsConstants.ADAPTIVE_TIMESTEP_NEAR_COLLISION);
    }
    
    public RK4Integrator(PhysicsModel physicsModel) {
        this(physicsModel, PhysicsConstants.DEFAULT_TIMESTEP);
    }
    
    /**
     * Create integrator with adaptive timestep support.
     */
    public RK4Integrator(PhysicsModel physicsModel, double normalTimestep, 
                         boolean adaptiveTimestep, double fineTimestep) {
        this.physicsModel = physicsModel;
        this.timestep = normalTimestep;
        this.adaptiveTimestep = adaptiveTimestep;
        this.fineTimestep = fineTimestep;
        this.collisionProximityThreshold = physicsModel.getProjectile().radius * 3.0;
    }
    
    /**
     * Create adaptive RK4 integrator.
     */
    public static RK4Integrator createAdaptive(PhysicsModel physicsModel) {
        return new RK4Integrator(physicsModel, 
            PhysicsConstants.DEFAULT_TIMESTEP, 
            true, 
            PhysicsConstants.ADAPTIVE_TIMESTEP_NEAR_COLLISION);
    }
    
    /**
     * Perform one RK4 integration step.
     * Returns the new state after time dt.
     */
    public ProjectileState step(ProjectileState state) {
        double dt = adaptiveTimestep ? selectTimestep(state) : timestep;
        return stepWithTimestep(state, dt);
    }
    
    /**
     * Select appropriate timestep based on proximity to surfaces.
     */
    protected double selectTimestep(ProjectileState state) {
        if (!adaptiveTimestep) {
            return timestep;
        }
        
        // Check distance to HUB opening plane
        double distToPlane = Math.abs(state.position.z - PhysicsConstants.DEFAULT_HUB_OPENING_HEIGHT);
        
        if (distToPlane < collisionProximityThreshold) {
            return fineTimestep;
        }
        
        return timestep;
    }
    
    /**
     * Perform RK4 step with specified timestep.
     */
    protected ProjectileState stepWithTimestep(ProjectileState state, double dt) {
        // k1 = f(t, y)
        StateDerivative k1 = physicsModel.computeDerivative(state);
        
        // k2 = f(t + dt/2, y + k1*dt/2)
        ProjectileState state2 = applyDerivative(state, k1, dt / 2.0);
        StateDerivative k2 = physicsModel.computeDerivative(state2);
        
        // k3 = f(t + dt/2, y + k2*dt/2)
        ProjectileState state3 = applyDerivative(state, k2, dt / 2.0);
        StateDerivative k3 = physicsModel.computeDerivative(state3);
        
        // k4 = f(t + dt, y + k3*dt)
        ProjectileState state4 = applyDerivative(state, k3, dt);
        StateDerivative k4 = physicsModel.computeDerivative(state4);
        
        // y_new = y + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
        StateDerivative combined = k1
            .add(k2.scale(2.0))
            .add(k3.scale(2.0))
            .add(k4)
            .scale(dt / 6.0);
        
        Vector3D newPosition = state.position.add(combined.positionDerivative);
        Vector3D newVelocity = state.velocity.add(combined.velocityDerivative);
        Vector3D newSpin = state.spin.add(combined.spinDerivative);
        double newTime = state.time + dt;
        
        return new ProjectileState(newPosition, newVelocity, newSpin, newTime);
    }
    
    /**
     * Apply a derivative to a state for a given time step.
     */
    protected ProjectileState applyDerivative(ProjectileState state, StateDerivative derivative, double dt) {
        Vector3D newPosition = state.position.add(derivative.positionDerivative.scale(dt));
        Vector3D newVelocity = state.velocity.add(derivative.velocityDerivative.scale(dt));
        Vector3D newSpin = state.spin.add(derivative.spinDerivative.scale(dt));
        return new ProjectileState(newPosition, newVelocity, newSpin, state.time + dt);
    }
    
    public double getTimestep() {
        return timestep;
    }
    
    public boolean isAdaptive() {
        return adaptiveTimestep;
    }
}
