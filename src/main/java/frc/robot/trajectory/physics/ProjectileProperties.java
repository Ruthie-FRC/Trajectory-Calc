package frc.robot.trajectory.physics;


/**
 * Runtime-configurable projectile properties.
 * Allows different ball masses, diameters, and physical characteristics.
 */
public class ProjectileProperties {
    public final double diameter; // meters
    public final double radius; // meters
    public final double massKg; // kilograms
    public final double crossSectionalArea; // m^2
    
    /**
     * Create projectile properties with default FRC ball specifications.
     */
    public ProjectileProperties() {
        this(PhysicsConstants.DEFAULT_BALL_DIAMETER, PhysicsConstants.DEFAULT_BALL_MASS_KG);
    }
    
    /**
     * Create projectile properties with custom diameter and mass.
     * 
     * @param diameter Ball diameter in meters
     * @param massKg Ball mass in kilograms
     */
    public ProjectileProperties(double diameter, double massKg) {
        if (diameter <= 0.0 || diameter > 1.0) {
            throw new IllegalArgumentException("Ball diameter must be between 0 and 1 meter");
        }
        if (massKg <= 0.0 || massKg > 10.0) {
            throw new IllegalArgumentException("Ball mass must be between 0 and 10 kg");
        }
        
        this.diameter = diameter;
        this.radius = diameter / 2.0;
        this.massKg = massKg;
        this.crossSectionalArea = Math.PI * radius * radius;
    }
    
    /**
     * Create projectile properties with mass specified in pounds.
     */
    public static ProjectileProperties fromPounds(double diameter, double massLb) {
        return new ProjectileProperties(diameter, massLb * PhysicsConstants.LB_TO_KG);
    }
    
    /**
     * Create properties for a worn ball (slightly lighter and smaller).
     */
    public static ProjectileProperties wornBall() {
        return new ProjectileProperties(
            PhysicsConstants.DEFAULT_BALL_DIAMETER * 0.98, // 2% smaller
            PhysicsConstants.DEFAULT_BALL_MASS_KG * 0.95   // 5% lighter
        );
    }
    
    /**
     * Create properties for a new/heavy ball.
     */
    public static ProjectileProperties heavyBall() {
        return fromPounds(
            PhysicsConstants.DEFAULT_BALL_DIAMETER,
            PhysicsConstants.DEFAULT_BALL_MASS_LB_MAX
        );
    }
    
    /**
     * Create properties for a light ball.
     */
    public static ProjectileProperties lightBall() {
        return fromPounds(
            PhysicsConstants.DEFAULT_BALL_DIAMETER,
            PhysicsConstants.DEFAULT_BALL_MASS_LB_MIN
        );
    }
    
    @Override
    public String toString() {
        return String.format("ProjectileProperties[d=%.3fm, m=%.3fkg, A=%.6fm²]",
            diameter, massKg, crossSectionalArea);
    }
}
