package frc.robot.trajectory;

/**
 * Complete state of the projectile including position, velocity, and spin.
 */
public class ProjectileState {
    public final Vector3D position;     // meters
    public final Vector3D velocity;     // m/s
    public final Vector3D spin;         // rad/s
    public final double time;           // seconds
    
    public ProjectileState(Vector3D position, Vector3D velocity, Vector3D spin, double time) {
        this.position = position;
        this.velocity = velocity;
        this.spin = spin;
        this.time = time;
    }
    
    public ProjectileState(Vector3D position, Vector3D velocity, Vector3D spin) {
        this(position, velocity, spin, 0.0);
    }
    
    /**
     * Create a new state with updated values.
     */
    public ProjectileState withPosition(Vector3D newPosition) {
        return new ProjectileState(newPosition, velocity, spin, time);
    }
    
    public ProjectileState withVelocity(Vector3D newVelocity) {
        return new ProjectileState(position, newVelocity, spin, time);
    }
    
    public ProjectileState withSpin(Vector3D newSpin) {
        return new ProjectileState(position, velocity, newSpin, time);
    }
    
    public ProjectileState withTime(double newTime) {
        return new ProjectileState(position, velocity, spin, newTime);
    }
    
    @Override
    public String toString() {
        return String.format("State[t=%.3f, pos=%s, vel=%s, spin=%s]",
            time, position, velocity, spin);
    }
}
