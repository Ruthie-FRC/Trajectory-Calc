package com.ruthiefrc.trajectory.advantagekit;

import com.ruthiefrc.trajectory.*;

/**
 * Helper class for integrating trajectory calculator with AdvantageKit logging.
 * 
 * This class simplifies the integration by providing a single place to:
 * - Calculate trajectories
 * - Log all relevant data to AdvantageKit
 * - Track shots for calibration
 * 
 * Usage:
 * <pre>
 * {@code
 * // In your subsystem
 * private final AdvantageKitTrajectoryHelper trajectoryHelper;
 * 
 * public ShooterSubsystem() {
 *     trajectoryHelper = new AdvantageKitTrajectoryHelper("Shooter/Trajectory");
 * }
 * 
 * public void aimAndShoot(Pose2d robotPose) {
 *     // Calculate and log
 *     InverseSolver.SolutionResult solution = trajectoryHelper.calculateAndLog(
 *         robotPose.getX(), robotPose.getY(), SHOOTER_HEIGHT, LAUNCH_SPEED, SPIN_RATE
 *     );
 *     
 *     // Use solution to aim
 *     setTurretAngle(solution.launchYawDeg);
 *     setHoodAngle(solution.launchPitchDeg);
 *     
 *     // After shooting, log result
 *     trajectoryHelper.logShotResult(robotPose.getX(), robotPose.getY(), solution, hit);
 * }
 * }
 * </pre>
 */
public class AdvantageKitTrajectoryHelper {
    
    private final ShooterController controller;
    private final String logPrefix;
    private boolean loggingEnabled = true;
    
    /**
     * Create helper with default calibration parameters.
     * 
     * @param logPrefix Prefix for AdvantageKit logs (e.g., "Shooter/Trajectory")
     */
    public AdvantageKitTrajectoryHelper(String logPrefix) {
        this(logPrefix, new CalibrationParameters());
    }
    
    /**
     * Create helper with custom calibration parameters.
     * 
     * @param logPrefix Prefix for AdvantageKit logs
     * @param params Initial calibration parameters
     */
    public AdvantageKitTrajectoryHelper(String logPrefix, CalibrationParameters params) {
        this.controller = new ShooterController(params);
        this.logPrefix = logPrefix.endsWith("/") ? logPrefix : logPrefix + "/";
        
        // Check if AdvantageKit is available
        try {
            Class.forName("org.littletonrobotics.junction.Logger");
        } catch (ClassNotFoundException e) {
            loggingEnabled = false;
            System.err.println("WARNING: AdvantageKit not found, logging disabled");
        }
    }
    
    /**
     * Calculate trajectory with full spin vector control and automatically log to AdvantageKit.
     * 
     * @param robotX Robot X position (meters)
     * @param robotY Robot Y position (meters)
     * @param shooterHeight Height of shooter above ground (meters)
     * @param launchSpeed Commanded launch speed (m/s)
     * @param spinRate Spin rate (rad/s)
     * @param spinAxisX Spin axis X component
     * @param spinAxisY Spin axis Y component
     * @param spinAxisZ Spin axis Z component
     * @return Solution with launch angles
     */
    public InverseSolver.SolutionResult calculateAndLog(double robotX, double robotY, double shooterHeight,
                                                         double launchSpeed, double spinRate,
                                                         double spinAxisX, double spinAxisY, double spinAxisZ) {
        // Time the calculation
        long startTime = System.currentTimeMillis();
        
        // Calculate solution
        InverseSolver.SolutionResult solution = controller.calculateShot(
            robotX, robotY, shooterHeight, launchSpeed, spinRate, spinAxisX, spinAxisY, spinAxisZ
        );
        
        long calcTime = System.currentTimeMillis() - startTime;
        
        // Log to AdvantageKit
        if (loggingEnabled) {
            logInputs(robotX, robotY, shooterHeight, launchSpeed, spinRate, spinAxisX, spinAxisY, spinAxisZ);
            logOutputs(solution, calcTime);
            logCalibration();
        }
        
        return solution;
    }
    
    /**
     * Calculate trajectory and automatically log all data to AdvantageKit.
     * Uses default backspin (Y-axis).
     * 
     * @param robotX Robot X position (meters)
     * @param robotY Robot Y position (meters)
     * @param shooterHeight Height of shooter above ground (meters)
     * @param launchSpeed Commanded launch speed (m/s)
     * @param spinRate Spin rate (rad/s)
     * @return Solution with launch angles
     */
    public InverseSolver.SolutionResult calculateAndLog(double robotX, double robotY, double shooterHeight,
                                                         double launchSpeed, double spinRate) {
        // Default: backspin around Y-axis
        return calculateAndLog(robotX, robotY, shooterHeight, launchSpeed, spinRate, 0, 1, 0);
    }
    
    /**
     * Log the result of a shot for calibration.
     * Call this after the shot lands and you know if it hit.
     * 
     * @param robotX Robot X position when shot was taken
     * @param robotY Robot Y position when shot was taken
     * @param solution The solution that was used
     * @param launchSpeed Launch speed used
     * @param spinRate Spin rate used
     * @param hit True if shot scored, false if missed
     */
    public void logShotResult(double robotX, double robotY, InverseSolver.SolutionResult solution, 
                               double launchSpeed, double spinRate, boolean hit) {
        // Log to calibration system
        controller.logShot(
            robotX, robotY, 0.0,
            solution, launchSpeed, spinRate, hit
        );
        
        // Log result to AdvantageKit
        if (loggingEnabled) {
            logToAdvantageKit(logPrefix + "Shot/Result/Hit", hit);
            logToAdvantageKit(logPrefix + "Shot/Result/Timestamp", System.currentTimeMillis());
        }
    }
    
    /**
     * Run calibration based on logged shots.
     * 
     * @return True if calibration was performed and parameters updated
     */
    public boolean calibrate() {
        boolean updated = controller.calibrate();
        
        if (updated && loggingEnabled) {
            logCalibration();
            logToAdvantageKit(logPrefix + "Calibration/Updated", true);
        }
        
        return updated;
    }
    
    /**
     * Get current calibration parameters.
     */
    public CalibrationParameters getCalibrationParameters() {
        return controller.getCalibrationParameters();
    }
    
    /**
     * Get calibration statistics.
     */
    public String getCalibrationStats() {
        return controller.getCalibrationStats();
    }
    
    /**
     * Update calibration parameters manually.
     */
    public void updateCalibration(CalibrationParameters params) {
        controller.updateCalibration(params);
        
        if (loggingEnabled) {
            logCalibration();
        }
    }
    
    /**
     * Get the underlying ShooterController for advanced usage.
     */
    public ShooterController getController() {
        return controller;
    }
    
    /**
     * Enable or disable AdvantageKit logging.
     */
    public void setLoggingEnabled(boolean enabled) {
        this.loggingEnabled = enabled && isAdvantageKitAvailable();
    }
    
    /**
     * Check if AdvantageKit is available.
     */
    public boolean isAdvantageKitAvailable() {
        try {
            Class.forName("org.littletonrobotics.junction.Logger");
            return true;
        } catch (ClassNotFoundException e) {
            return false;
        }
    }
    
    // Private logging methods
    
    private void logInputs(double x, double y, double z, double speed, double spin,
                          double spinAxisX, double spinAxisY, double spinAxisZ) {
        logToAdvantageKit(logPrefix + "Input/RobotX", x);
        logToAdvantageKit(logPrefix + "Input/RobotY", y);
        logToAdvantageKit(logPrefix + "Input/ShooterHeight", z);
        logToAdvantageKit(logPrefix + "Input/LaunchSpeed", speed);
        logToAdvantageKit(logPrefix + "Input/SpinRate", spin);
        logToAdvantageKit(logPrefix + "Input/SpinAxisX", spinAxisX);
        logToAdvantageKit(logPrefix + "Input/SpinAxisY", spinAxisY);
        logToAdvantageKit(logPrefix + "Input/SpinAxisZ", spinAxisZ);
        logToAdvantageKit(logPrefix + "Input/Timestamp", System.currentTimeMillis());
    }
    
    private void logInputs(double x, double y, double z, double speed, double spin) {
        // Default backspin
        logInputs(x, y, z, speed, spin, 0, 1, 0);
    }
    
    private void logOutputs(InverseSolver.SolutionResult solution, long calcTime) {
        if (solution != null) {
            logToAdvantageKit(logPrefix + "Output/LaunchYawDeg", solution.launchYawDeg);
            logToAdvantageKit(logPrefix + "Output/LaunchPitchDeg", solution.launchPitchDeg);
            logToAdvantageKit(logPrefix + "Output/Score", solution.score);
            logToAdvantageKit(logPrefix + "Output/IsHit", solution.isHit());
            logToAdvantageKit(logPrefix + "Output/CalculationTimeMs", calcTime);
            
            if (solution.trajectory != null) {
                logToAdvantageKit(logPrefix + "Output/FlightTime", solution.trajectory.getFlightTime());
                logToAdvantageKit(logPrefix + "Output/MaxHeight", solution.trajectory.getMaxHeight());
                logToAdvantageKit(logPrefix + "Output/TrajectoryPoints", solution.trajectory.trajectory.size());
            }
        }
    }
    
    private void logCalibration() {
        CalibrationParameters params = controller.getCalibrationParameters();
        logToAdvantageKit(logPrefix + "Calibration/DragCoeff", params.dragCoefficient);
        logToAdvantageKit(logPrefix + "Calibration/MagnusCoeff", params.magnusCoefficient);
        logToAdvantageKit(logPrefix + "Calibration/SpeedEfficiency", params.speedEfficiency);
        logToAdvantageKit(logPrefix + "Calibration/SpinEfficiency", params.spinEfficiency);
        logToAdvantageKit(logPrefix + "Calibration/SpinDecayRate", params.spinDecayRate);
        logToAdvantageKit(logPrefix + "Calibration/Restitution", params.restitutionCoefficient);
        logToAdvantageKit(logPrefix + "Calibration/Friction", params.frictionCoefficient);
    }
    
    private void logToAdvantageKit(String key, double value) {
        try {
            Class<?> loggerClass = Class.forName("org.littletonrobotics.junction.Logger");
            java.lang.reflect.Method recordOutput = loggerClass.getMethod("recordOutput", String.class, double.class);
            recordOutput.invoke(null, key, value);
        } catch (Exception e) {
            // Silently fail - AdvantageKit not available
        }
    }
    
    private void logToAdvantageKit(String key, boolean value) {
        try {
            Class<?> loggerClass = Class.forName("org.littletonrobotics.junction.Logger");
            java.lang.reflect.Method recordOutput = loggerClass.getMethod("recordOutput", String.class, boolean.class);
            recordOutput.invoke(null, key, value);
        } catch (Exception e) {
            // Silently fail - AdvantageKit not available
        }
    }
    
    private void logToAdvantageKit(String key, long value) {
        try {
            Class<?> loggerClass = Class.forName("org.littletonrobotics.junction.Logger");
            java.lang.reflect.Method recordOutput = loggerClass.getMethod("recordOutput", String.class, long.class);
            recordOutput.invoke(null, key, value);
        } catch (Exception e) {
            // Silently fail - AdvantageKit not available
        }
    }
}
