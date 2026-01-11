package examples.wpilib;

import com.ruthiefrc.trajectory.*;
import com.ruthiefrc.trajectory.advantagekit.TrajectoryLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Example WPILib shooter subsystem integrating the trajectory calculator.
 * 
 * This shows how to integrate the ballistic solver into a standard WPILib robot
 * with full AdvantageKit logging support.
 * 
 * Adapt motor types, sensor inputs, and control logic to your specific robot.
 * 
 * Integration steps:
 * 1. Add TrajectoryCalc.json to your vendordeps folder
 * 2. Copy this class to your robot project
 * 3. Modify motor controllers and hardware interfaces
 * 4. Add to your RobotContainer
 */
public class ExampleShooterSubsystem extends SubsystemBase {
    
    // Trajectory calculator - create once, reuse throughout match
    private final ShooterController trajectoryCalc;
    
    // Last calculated solution (cached for logging)
    private InverseSolver.SolutionResult lastSolution;
    
    // AdvantageKit logging support
    private boolean advantageKitEnabled = false;
    private TrajectoryLogger.Inputs logInputs = new TrajectoryLogger.Inputs();
    private TrajectoryLogger.Outputs logOutputs = new TrajectoryLogger.Outputs();
    private TrajectoryLogger.Calibration logCalibration = new TrajectoryLogger.Calibration();
    
    // Physical constants for your robot
    private static final double SHOOTER_HEIGHT_METERS = 0.8;  // Height above ground
    private static final double NOMINAL_LAUNCH_SPEED = 12.0;   // m/s
    private static final double SPIN_RATE = 100.0;             // rad/s
    
    public ExampleShooterSubsystem() {
        // Initialize with default calibration parameters
        // Can also load custom parameters from file or NetworkTables
        trajectoryCalc = new ShooterController();
        
        // Alternative: custom calibration
        // CalibrationParameters customParams = new CalibrationParameters()
        //     .withSpeedEfficiency(0.92)
        //     .withDragCoefficient(0.45);
        // trajectoryCalc = new ShooterController(customParams);
        
        // Enable AdvantageKit logging if available
        try {
            Class.forName("org.littletonrobotics.junction.Logger");
            advantageKitEnabled = true;
        } catch (ClassNotFoundException e) {
            advantageKitEnabled = false;
        }
    }
    
    /**
     * Calculate shooting solution for current robot position.
     * Call this periodically or when you need updated aim angles.
     * 
     * @param robotPose Current robot pose from odometry or vision
     * @return Solution with launch angles, or null if no solution
     */
    public InverseSolver.SolutionResult calculateShot(Pose2d robotPose) {
        double x = robotPose.getX();
        double y = robotPose.getY();
        double z = SHOOTER_HEIGHT_METERS;
        
        // Create AdvantageKit inputs
        logInputs = TrajectoryLogger.createInputs(x, y, z, 0, 0, 
            PhysicsConstants.HUB_CENTER_Z, NOMINAL_LAUNCH_SPEED, SPIN_RATE);
        
        // Time the calculation
        long startTime = System.currentTimeMillis();
        
        // Calculate solution
        lastSolution = trajectoryCalc.calculateShot(x, y, z, NOMINAL_LAUNCH_SPEED, SPIN_RATE);
        
        long calcTime = System.currentTimeMillis() - startTime;
        
        // Create AdvantageKit outputs
        logOutputs = TrajectoryLogger.toOutputs(lastSolution, calcTime);
        
        // Update calibration log data
        CalibrationParameters params = trajectoryCalc.getCalibrationParameters();
        // Note: Shot count and hit rate would come from calibration system
        logCalibration = TrajectoryLogger.toCalibration(params, 0, 0.0);
        
        return lastSolution;
    }
    
    /**
     * Check if target is reachable before attempting full calculation.
     * Use this for early rejection or to disable shooting UI.
     */
    public boolean canReachTarget(Pose2d robotPose) {
        return trajectoryCalc.canReachTarget(
            robotPose.getX(),
            robotPose.getY(),
            SHOOTER_HEIGHT_METERS,
            NOMINAL_LAUNCH_SPEED
        );
    }
    
    /**
     * Apply calculated solution to robot mechanisms.
     * Implement this based on your robot's hardware.
     */
    public void aimAtTarget(InverseSolver.SolutionResult solution) {
        if (solution == null || !solution.isHit()) {
            // No valid solution
            return;
        }
        
        // Convert yaw to robot-relative angle
        // NOTE: You may need to adjust this based on your coordinate system
        double turretAngleDeg = solution.launchYawDeg;
        double hoodAngleDeg = solution.launchPitchDeg;
        
        // Set turret position
        // turretMotor.setPosition(turretAngleDeg);
        
        // Set hood angle  
        // hoodMotor.setPosition(hoodAngleDeg);
        
        // Spin up flywheel to launch speed
        // flywheelMotor.setVelocity(NOMINAL_LAUNCH_SPEED);
    }
    
    /**
     * Log a shot for calibration.
     * Call this after each shot attempt with the actual result.
     * 
     * @param robotPose Robot position when shot was taken
     * @param hit True if shot scored, false if missed
     */
    public void logShotResult(Pose2d robotPose, boolean hit) {
        if (lastSolution != null) {
            trajectoryCalc.logShot(
                robotPose.getX(),
                robotPose.getY(),
                SHOOTER_HEIGHT_METERS,
                lastSolution,
                NOMINAL_LAUNCH_SPEED,
                SPIN_RATE,
                hit
            );
        }
    }
    
    /**
     * Perform calibration using logged shot data.
     * Call this periodically (e.g., in autonomous init or test mode).
     * 
     * @return True if calibration was performed
     */
    public boolean performCalibration() {
        return trajectoryCalc.calibrate();
    }
    
    /**
     * Get calibration statistics for dashboard display.
     */
    public String getCalibrationStats() {
        return trajectoryCalc.getCalibrationStats();
    }
    
    /**
     * Simulate a specific trajectory for testing.
     * Useful for validation and debugging.
     */
    public TrajectorySimulator.TrajectoryResult testTrajectory(
            Pose2d robotPose, double yawDeg, double pitchDeg) {
        return trajectoryCalc.simulateTrajectory(
            robotPose.getX(),
            robotPose.getY(),
            SHOOTER_HEIGHT_METERS,
            NOMINAL_LAUNCH_SPEED,
            yawDeg,
            pitchDeg,
            SPIN_RATE
        );
    }
    
    @Override
    public void periodic() {
        // Update dashboard with calibration info
        SmartDashboard.putString("Shooter/Calibration", getCalibrationStats());
        
        // Update AdvantageKit logs if enabled
        if (advantageKitEnabled) {
            updateAdvantageKitLogs();
        }
    }
    
    /**
     * Update AdvantageKit logs with latest data.
     * Uses reflection to avoid hard dependency on AdvantageKit.
     */
    private void updateAdvantageKitLogs() {
        try {
            Class<?> loggerClass = Class.forName("org.littletonrobotics.junction.Logger");
            java.lang.reflect.Method recordOutput = loggerClass.getMethod("recordOutput", String.class, double.class);
            java.lang.reflect.Method recordOutputBoolean = loggerClass.getMethod("recordOutput", String.class, boolean.class);
            java.lang.reflect.Method recordOutputInt = loggerClass.getMethod("recordOutput", String.class, long.class);
            
            // Log inputs
            recordOutput.invoke(null, "Trajectory/Inputs/RobotX", logInputs.robotX);
            recordOutput.invoke(null, "Trajectory/Inputs/RobotY", logInputs.robotY);
            recordOutput.invoke(null, "Trajectory/Inputs/RobotZ", logInputs.robotZ);
            recordOutput.invoke(null, "Trajectory/Inputs/LaunchSpeed", logInputs.launchSpeed);
            recordOutput.invoke(null, "Trajectory/Inputs/SpinRate", logInputs.spinRate);
            
            // Log outputs
            recordOutput.invoke(null, "Trajectory/Outputs/LaunchYawDeg", logOutputs.launchYawDeg);
            recordOutput.invoke(null, "Trajectory/Outputs/LaunchPitchDeg", logOutputs.launchPitchDeg);
            recordOutput.invoke(null, "Trajectory/Outputs/Score", logOutputs.solutionScore);
            recordOutputBoolean.invoke(null, "Trajectory/Outputs/IsHit", logOutputs.isHit);
            recordOutput.invoke(null, "Trajectory/Outputs/FlightTime", logOutputs.flightTime);
            recordOutput.invoke(null, "Trajectory/Outputs/MaxHeight", logOutputs.maxHeight);
            recordOutputInt.invoke(null, "Trajectory/Outputs/CalculationTimeMs", logOutputs.calculationTimeMs);
            
            // Log calibration
            recordOutput.invoke(null, "Trajectory/Calibration/DragCoeff", logCalibration.dragCoefficient);
            recordOutput.invoke(null, "Trajectory/Calibration/SpeedEfficiency", logCalibration.speedEfficiency);
            recordOutputInt.invoke(null, "Trajectory/Calibration/ShotCount", (long)logCalibration.shotCount);
            recordOutput.invoke(null, "Trajectory/Calibration/HitRate", logCalibration.hitRate);
            
        } catch (Exception e) {
            // AdvantageKit not available or error logging - silently continue
        }
    }
}
