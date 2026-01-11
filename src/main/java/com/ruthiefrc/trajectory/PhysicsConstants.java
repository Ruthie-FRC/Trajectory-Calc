package com.ruthiefrc.trajectory;

/**
 * Physical constants and projectile properties for FRC ballistic calculations.
 */
public class PhysicsConstants {
    
    // Physical constants
    public static final double GRAVITY = 9.80665; // m/s^2 (standard gravity)
    public static final double AIR_DENSITY = 1.225; // kg/m^3 (sea level, 15°C)
    
    // Projectile properties
    public static final double BALL_DIAMETER = 0.15; // meters
    public static final double BALL_RADIUS = 0.075; // meters
    public static final double BALL_MASS_LB_MIN = 0.448; // pounds
    public static final double BALL_MASS_LB_MAX = 0.500; // pounds
    public static final double BALL_MASS_LB_NOMINAL = 0.474; // pounds
    public static final double BALL_MASS_KG = 0.474 * 0.453592; // kg (nominal)
    public static final double BALL_CROSS_SECTION = Math.PI * BALL_RADIUS * BALL_RADIUS; // m^2
    
    // Drag model for sphere
    public static final double DRAG_COEFFICIENT = 0.47; // typical for smooth sphere
    
    // Magnus effect coefficient (empirical, tunable)
    public static final double MAGNUS_COEFFICIENT = 0.00001; // tunable parameter
    
    // Collision properties
    public static final double RESTITUTION_COEFFICIENT = 0.6; // bounce coefficient
    public static final double FRICTION_COEFFICIENT = 0.3; // rolling/sliding friction
    
    // Shooter efficiency (tunable)
    public static final double SPEED_EFFICIENCY = 0.95; // launch speed efficiency
    public static final double SPIN_EFFICIENCY = 0.90; // spin efficiency
    
    // HUB target geometry
    public static final double HUB_FOOTPRINT_SIZE = 1.19; // meters (square footprint)
    public static final double HUB_OPENING_FLAT_TO_FLAT = 1.06; // meters (hexagon)
    public static final double HUB_OPENING_HEIGHT = 1.83; // meters above carpet
    public static final double HUB_CENTER_X = 0.0; // field coordinate (configurable)
    public static final double HUB_CENTER_Y = 0.0; // field coordinate (configurable)
    public static final double HUB_CENTER_Z = HUB_OPENING_HEIGHT; // center of opening
    
    // Hit acceptance criteria
    public static final double MIN_CLEARANCE_MARGIN = BALL_RADIUS * 1.2; // safety margin
    public static final double MAX_ENTRY_ANGLE_DEG = 45.0; // prefer downward entry
    public static final double MAX_VERTICAL_VELOCITY = 3.0; // m/s to reduce bounce-out
    
    // Numerical integration
    public static final double DEFAULT_TIMESTEP = 0.001; // seconds (1ms)
    public static final double MAX_SIMULATION_TIME = 5.0; // seconds
    public static final double CONVERGENCE_TOLERANCE = 0.01; // meters
    
    private PhysicsConstants() {
        // Prevent instantiation
    }
}
