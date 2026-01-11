package frc.robot.trajectory;

import frc.robot.trajectory.calibration.CalibrationParameters;
import frc.robot.trajectory.calibration.CalibrationSystem;
import frc.robot.trajectory.physics.HubGeometry;
import frc.robot.trajectory.physics.PhysicsConstants;
import frc.robot.trajectory.physics.PhysicsModel;
import frc.robot.trajectory.physics.ProjectileProperties;
import frc.robot.trajectory.simulation.TrajectorySimulator;
import frc.robot.trajectory.solver.InverseSolver;
import frc.robot.trajectory.util.Vector3D;

/**
 * Main facade for the trajectory calculation system.
 * Provides a clean interface for robot code to compute shooting parameters.
 * Supports runtime-configurable parameters and optional logging.
 */
public class ShooterController {
    private PhysicsModel physicsModel;
    private final ProjectileProperties projectile;
    private final HubGeometry hubGeometry;
    private TrajectorySimulator simulator;
    private InverseSolver solver;
    private final CalibrationSystem calibration;
    private boolean loggingEnabled;
    
    public ShooterController() {
        this(new CalibrationParameters(), new ProjectileProperties());
    }
    
    public ShooterController(CalibrationParameters initialParams) {
        this(initialParams, new ProjectileProperties());
    }
    
    public ShooterController(CalibrationParameters initialParams, ProjectileProperties projectile) {
        this.projectile = projectile;
        this.calibration = new CalibrationSystem(initialParams);
        this.physicsModel = new PhysicsModel(initialParams, projectile);
        this.hubGeometry = new HubGeometry();
        this.simulator = new TrajectorySimulator(physicsModel, hubGeometry);
        this.solver = new InverseSolver(simulator);
        this.loggingEnabled = true; // Default: logging enabled
    }
    
    /**
     * Enable or disable shot logging at runtime.
     * Useful for reducing overhead during competition.
     */
    public void setLoggingEnabled(boolean enabled) {
        this.loggingEnabled = enabled;
    }
    
    public boolean isLoggingEnabled() {
        return loggingEnabled;
    }
    
    /**
     * Calculate shooting parameters for a given robot position with full spin vector control.
     * 
     * @param robotX X position in meters
     * @param robotY Y position in meters
     * @param robotZ Z position (shooter height) in meters
     * @param nominalSpeed Desired launch speed in m/s
     * @param spinRate Spin rate in rad/s
     * @param spinAxisX Spin axis X component (will be normalized)
     * @param spinAxisY Spin axis Y component (will be normalized)
     * @param spinAxisZ Spin axis Z component (will be normalized)
     * @return Solution with launch angles and quality score
     */
    public InverseSolver.SolutionResult calculateShot(double robotX, double robotY, double robotZ,
                                                       double nominalSpeed, double spinRate,
                                                       double spinAxisX, double spinAxisY, double spinAxisZ) {
        Vector3D robotPosition = new Vector3D(robotX, robotY, robotZ);
        
        // Normalize spin axis and apply spin rate
        Vector3D spinAxis = new Vector3D(spinAxisX, spinAxisY, spinAxisZ);
        double axisMagnitude = spinAxis.magnitude();
        if (axisMagnitude > 1e-6) {
            spinAxis = spinAxis.scale(1.0 / axisMagnitude);
        } else {
            // Default to backspin if no axis specified
            spinAxis = new Vector3D(0, 1, 0);
        }
        Vector3D spin = spinAxis.scale(spinRate);
        
        return solver.solve(robotPosition, nominalSpeed, spin);
    }
    
    /**
     * Calculate shooting parameters for a given robot position.
     * Uses default backspin around Y-axis.
     * 
     * @param robotX X position in meters
     * @param robotY Y position in meters
     * @param robotZ Z position (shooter height) in meters
     * @param nominalSpeed Desired launch speed in m/s
     * @param spinRate Spin rate in rad/s
     * @return Solution with launch angles and quality score
     */
    public InverseSolver.SolutionResult calculateShot(double robotX, double robotY, double robotZ,
                                                       double nominalSpeed, double spinRate) {
        // Default: backspin around Y-axis
        return calculateShot(robotX, robotY, robotZ, nominalSpeed, spinRate, 0, 1, 0);
    }
    
    /**
     * Calculate shooting parameters with default spin rate.
     */
    public InverseSolver.SolutionResult calculateShot(double robotX, double robotY, double robotZ,
                                                       double nominalSpeed) {
        return calculateShot(robotX, robotY, robotZ, nominalSpeed, 100.0); // Default spin
    }
    
    /**
     * Simulate a trajectory with given parameters (for testing/validation).
     */
    public TrajectorySimulator.TrajectoryResult simulateTrajectory(double robotX, double robotY, double robotZ,
                                                                     double launchSpeed, double yawDeg, 
                                                                     double pitchDeg, double spinRate) {
        Vector3D robotPosition = new Vector3D(robotX, robotY, robotZ);
        return simulator.simulateWithShooterModel(robotPosition, launchSpeed, yawDeg, pitchDeg, spinRate);
    }
    
    /**
     * Enable or disable incremental calibration.
     * When enabled, parameters are updated after each logged shot.
     * Use during practice; disable during competition.
     */
    public void setIncrementalCalibrationEnabled(boolean enabled) {
        calibration.setIncrementalCalibrationEnabled(enabled);
    }
    
    public boolean isIncrementalCalibrationEnabled() {
        return calibration.isIncrementalCalibrationEnabled();
    }
    
    /**
     * Set learning rate for incremental calibration (0.0 - 1.0).
     * Higher values = faster adaptation but less stability.
     */
    public void setLearningRate(double rate) {
        calibration.setLearningRate(rate);
    }
    
    /**
     * Log a shot result for calibration.
     * Only logs if logging is enabled.
     * If incremental calibration is enabled, updates parameters automatically.
     */
    public void logShot(double robotX, double robotY, double robotZ,
                       InverseSolver.SolutionResult solution, double launchSpeed, 
                       double spinRate, boolean hit) {
        if (!loggingEnabled) {
            return; // Skip logging if disabled
        }
        
        Vector3D robotPose = new Vector3D(robotX, robotY, robotZ);
        calibration.logShot(robotPose, solution, launchSpeed, spinRate, hit);
        
        // If incremental calibration is enabled and parameters changed, update components
        CalibrationParameters newParams = calibration.getCurrentParameters();
        if (!newParams.equals(getCalibrationParameters()) && calibration.isIncrementalCalibrationEnabled()) {
            updateCalibration(newParams);
        }
    }
    
    /**
     * Perform calibration based on logged shots.
     * Returns true if calibration was performed (enough data available).
     */
    public boolean calibrate() {
        CalibrationParameters oldParams = calibration.getCurrentParameters();
        CalibrationParameters newParams = calibration.calibrate();
        
        if (!oldParams.equals(newParams)) {
            updateCalibration(newParams);
            return true;
        }
        return false;
    }
    
    /**
     * Update calibration parameters and reinitialize components.
     * This allows runtime tuning of physics parameters.
     */
    public void updateCalibration(CalibrationParameters params) {
        calibration.updateParameters(params);
        
        // Recreate physics components with new parameters
        this.physicsModel = new PhysicsModel(params, projectile);
        this.simulator = new TrajectorySimulator(physicsModel, hubGeometry);
        this.solver = new InverseSolver(simulator);
    }
    
    /**
     * Update projectile properties at runtime.
     * Use this when switching to worn balls or different ball types.
     */
    public void updateProjectileProperties(ProjectileProperties newProjectile) {
        CalibrationParameters currentParams = calibration.getCurrentParameters();
        this.physicsModel = new PhysicsModel(currentParams, newProjectile);
        this.simulator = new TrajectorySimulator(physicsModel, hubGeometry);
        this.solver = new InverseSolver(simulator);
    }
    
    /**
     * Get calibration statistics.
     */
    public String getCalibrationStats() {
        return String.format("Shots logged: %d, Hit rate: %.1f%%, Parameters: %s",
            calibration.getShotCount(),
            calibration.getHitRate() * 100,
            calibration.getCurrentParameters());
    }
    
    /**
     * Get current calibration parameters.
     */
    public CalibrationParameters getCalibrationParameters() {
        return calibration.getCurrentParameters();
    }
    
    /**
     * Get current projectile properties.
     */
    public ProjectileProperties getProjectileProperties() {
        return projectile;
    }
    
    /**
     * Check if a position can reach the target with given speed.
     * Quick feasibility check before full solve.
     */
    public boolean canReachTarget(double robotX, double robotY, double robotZ, double maxSpeed) {
        Vector3D robotPos = new Vector3D(robotX, robotY, robotZ);
        Vector3D target = hubGeometry.getCenter();
        
        double dx = target.x - robotPos.x;
        double dy = target.y - robotPos.y;
        double dz = target.z - robotPos.z;
        double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        
        // Simple ballistic range check (ignoring drag)
        // Maximum range for given speed occurs at 45 degrees
        double maxRange = maxSpeed * maxSpeed / PhysicsConstants.GRAVITY;
        
        return distance < maxRange * 0.8; // Conservative estimate with drag
    }
}
