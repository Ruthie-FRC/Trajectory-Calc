package com.ruthiefrc.trajectory;

/**
 * Physical constants and projectile properties for FRC ballistic calculations.
 * Runtime-configurable parameters are managed via ProjectileProperties and CalibrationParameters.
 */
public class PhysicsConstants {
    
    // Physical constants (fixed)
    public static final double GRAVITY = 9.80665; // m/s^2 (standard gravity)
    public static final double AIR_DENSITY = 1.225; // kg/m^3 (sea level, 15°C)
    public static final double LB_TO_KG = 0.453592; // conversion factor
    
    // Default projectile properties (can be overridden via ProjectileProperties)
    public static final double DEFAULT_BALL_DIAMETER = 0.15; // meters
    public static final double DEFAULT_BALL_RADIUS = 0.075; // meters
    public static final double DEFAULT_BALL_MASS_LB_MIN = 0.448; // pounds
    public static final double DEFAULT_BALL_MASS_LB_MAX = 0.500; // pounds
    public static final double DEFAULT_BALL_MASS_LB = 0.474; // pounds
    public static final double DEFAULT_BALL_MASS_KG = DEFAULT_BALL_MASS_LB * LB_TO_KG; // kg
    
    // Default calibration values
    public static final double DEFAULT_DRAG_COEFFICIENT = 0.47; // typical for smooth sphere
    public static final double DEFAULT_MAGNUS_COEFFICIENT = 0.00001; // tunable parameter
    public static final double DEFAULT_RESTITUTION_COEFFICIENT = 0.6; // bounce coefficient
    public static final double DEFAULT_FRICTION_COEFFICIENT = 0.3; // rolling/sliding friction
    public static final double DEFAULT_SPEED_EFFICIENCY = 0.95; // launch speed efficiency
    public static final double DEFAULT_SPIN_EFFICIENCY = 0.90; // spin efficiency
    
    // Spin physics for foam balls (NEW)
    public static final double DEFAULT_SPIN_DECAY_RATE = 0.05; // per second for foam balls
    public static final double SPIN_BOUNCE_REDUCTION_FACTOR = 0.7; // spin reduction on rim/surface contact
    public static final double SPIN_FRICTION_COUPLING = 0.15; // spin-to-tangential velocity coupling
    public static final double MAX_SPIN_RATE = 500.0; // rad/s safety limit
    
    // Spin physics tuning constants
    public static final double SPIN_DECAY_VELOCITY_FACTOR = 20.0; // m/s normalization for velocity-dependent decay
    public static final double SPIN_ALIGNMENT_THRESHOLD = 0.5; // normalized spin alignment for bonus/penalty
    public static final double SPIN_ALIGNMENT_BONUS = 0.2; // risk reduction for favorable backspin
    public static final double SPIN_ALIGNMENT_PENALTY = 0.3; // risk increase for forward spin
    public static final double COLLISION_IMPACT_SEVERITY_SCALE = 10.0; // m/s for impact normalization
    public static final double COLLISION_SEVERITY_FACTOR = 0.3; // impact severity contribution to spin loss
    public static final double FRICTION_TORQUE_SCALE = 0.1; // friction torque magnitude scaling
    
    // Parameter safety limits
    public static final double MIN_DRAG_COEFFICIENT = 0.1;
    public static final double MAX_DRAG_COEFFICIENT = 2.0;
    public static final double MIN_MAGNUS_COEFFICIENT = 0.0;
    public static final double MAX_MAGNUS_COEFFICIENT = 0.001;
    public static final double MIN_SPEED_EFFICIENCY = 0.5;
    public static final double MAX_SPEED_EFFICIENCY = 1.0;
    public static final double MIN_SPIN_EFFICIENCY = 0.5;
    public static final double MAX_SPIN_EFFICIENCY = 1.0;
    public static final double MIN_RESTITUTION = 0.0;
    public static final double MAX_RESTITUTION = 1.0;
    public static final double MIN_FRICTION = 0.0;
    public static final double MAX_FRICTION = 1.0;
    public static final double MIN_SPIN_DECAY_RATE = 0.0;
    public static final double MAX_SPIN_DECAY_RATE = 0.5; // per second
    
    // HUB target geometry (default values, can be configured via HubGeometry)
    public static final double DEFAULT_HUB_FOOTPRINT_SIZE = 1.19; // meters (square footprint)
    public static final double DEFAULT_HUB_OPENING_FLAT_TO_FLAT = 1.06; // meters (hexagon)
    public static final double DEFAULT_HUB_OPENING_HEIGHT = 1.83; // meters above carpet
    public static final double DEFAULT_HUB_CENTER_X = 0.0; // field coordinate (configurable)
    public static final double DEFAULT_HUB_CENTER_Y = 0.0; // field coordinate (configurable)
    
    // Hit acceptance criteria
    public static final double DEFAULT_MIN_CLEARANCE_MARGIN = DEFAULT_BALL_RADIUS * 1.5; // safety margin
    public static final double DEFAULT_MAX_ENTRY_ANGLE_DEG = 45.0; // prefer downward entry
    public static final double DEFAULT_MAX_VERTICAL_VELOCITY = 3.0; // m/s to reduce bounce-out
    public static final double DEFAULT_MAX_LATERAL_VELOCITY = 2.0; // m/s lateral velocity at entry
    
    // Rim proximity rejection (NEW)
    public static final double RIM_DANGER_ZONE = DEFAULT_BALL_RADIUS * 2.0; // reject if this close to rim
    public static final double BOUNCE_OUT_RISK_THRESHOLD = 0.3; // normalized risk score
    
    // Numerical integration
    public static final double DEFAULT_TIMESTEP = 0.001; // seconds (1ms)
    public static final double ADAPTIVE_TIMESTEP_NEAR_COLLISION = 0.0002; // 0.2ms near surfaces
    public static final double MAX_SIMULATION_TIME = 5.0; // seconds
    public static final double CONVERGENCE_TOLERANCE = 0.01; // meters
    
    // Inverse solver improvements (NEW)
    public static final int DEFAULT_MAX_ITERATIONS = 50;
    public static final int GRID_SEARCH_REFINEMENT_STEPS = 3;
    public static final double INITIAL_YAW_STEP = 2.0; // degrees
    public static final double INITIAL_PITCH_STEP = 1.0; // degrees
    public static final double FINE_TUNING_YAW_STEP = 0.2; // degrees
    public static final double FINE_TUNING_PITCH_STEP = 0.1; // degrees
    
    private PhysicsConstants() {
        // Prevent instantiation
    }
}
