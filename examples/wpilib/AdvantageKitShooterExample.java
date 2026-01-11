package examples.wpilib;

import frc.robot.trajectory.*;
import frc.robot.trajectory.advantagekit.AdvantageKitTrajectoryHelper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simplified shooter subsystem example using AdvantageKit helper.
 * 
 * This is the EASIEST way to integrate the trajectory calculator with AdvantageKit.
 * All logging is handled automatically by the helper class.
 * 
 * Quick integration steps:
 * 1. Copy this file to your robot project
 * 2. Replace hardware interfaces (motors, sensors) with your actual hardware
 * 3. Update constants for your robot
 * 4. Add to RobotContainer
 * 5. Logs will automatically appear in AdvantageScope!
 */
public class AdvantageKitShooterExample extends SubsystemBase {
    
    // Trajectory calculator with automatic AdvantageKit logging
    private final AdvantageKitTrajectoryHelper trajectoryHelper;
    
    // Your robot constants
    private static final double SHOOTER_HEIGHT = 0.8;  // meters
    private static final double LAUNCH_SPEED = 12.0;   // m/s
    private static final double SPIN_RATE = 100.0;     // rad/s
    
    // Last calculated solution (for logging results later)
    private InverseSolver.SolutionResult lastSolution;
    private Pose2d lastShotPose;
    
    public AdvantageKitShooterExample() {
        // Create helper with default parameters
        // Logs will appear under "Shooter/Trajectory/" in AdvantageScope
        trajectoryHelper = new AdvantageKitTrajectoryHelper("Shooter/Trajectory");
        
        // OR: Create with custom initial parameters if you already know them
        // CalibrationParameters customParams = new CalibrationParameters()
        //     .withSpeedEfficiency(0.92)
        //     .withDragCoefficient(0.45);
        // trajectoryHelper = new AdvantageKitTrajectoryHelper("Shooter/Trajectory", customParams);
    }
    
    /**
     * Calculate shot for current robot position.
     * Automatically logs all data to AdvantageKit.
     */
    public void calculateShot(Pose2d robotPose) {
        // This single call:
        // 1. Calculates the trajectory
        // 2. Logs inputs (robot position, speeds, etc.)
        // 3. Logs outputs (angles, quality, etc.)
        // 4. Logs calibration parameters
        lastSolution = trajectoryHelper.calculateAndLog(
            robotPose,
            SHOOTER_HEIGHT,
            LAUNCH_SPEED,
            SPIN_RATE
        );
        
        lastShotPose = robotPose;
    }
    
    /**
     * Aim shooter at target using calculated solution.
     */
    public void aimAtTarget() {
        if (lastSolution != null && lastSolution.isHit()) {
            // TODO: Replace with your actual hardware
            // setTurretAngle(lastSolution.launchYawDeg);
            // setHoodAngle(lastSolution.launchPitchDeg);
            // setFlywheelSpeed(LAUNCH_SPEED);
            
            System.out.println("Aiming: Yaw=" + lastSolution.launchYawDeg + 
                             "° Pitch=" + lastSolution.launchPitchDeg + "°");
        }
    }
    
    /**
     * Execute a full shot sequence.
     * Call this from a command.
     */
    public void shoot(Pose2d currentPose) {
        // 1. Calculate trajectory (automatically logs)
        calculateShot(currentPose);
        
        // 2. Aim
        aimAtTarget();
        
        // 3. Spin up flywheel
        // TODO: spinUpFlywheel();
        
        // 4. Wait for flywheel to reach speed
        // TODO: waitForFlywheelReady();
        
        // 5. Fire!
        // TODO: feedBall();
        
        System.out.println("Shot executed!");
    }
    
    /**
     * Log shot result after ball lands.
     * Call this 1-2 seconds after shooting, once you know if it scored.
     * 
     * @param hit True if the shot scored, false if it missed
     */
    public void logShotResult(boolean hit) {
        if (lastSolution != null && lastShotPose != null) {
            // This logs the result and adds to calibration database
            trajectoryHelper.logShotResult(
                lastShotPose,
                lastSolution,
                LAUNCH_SPEED,
                SPIN_RATE,
                hit
            );
            
            System.out.println("Shot result logged: " + (hit ? "HIT" : "MISS"));
        }
    }
    
    /**
     * Run calibration after practice.
     * Call this in test mode or disabled mode after collecting shot data.
     */
    public void runCalibration() {
        boolean updated = trajectoryHelper.calibrate();
        
        if (updated) {
            System.out.println("Calibration updated!");
            System.out.println(trajectoryHelper.getCalibrationStats());
            
            // Optionally save parameters to file or preferences
            // saveCalibrationToFile(trajectoryHelper.getCalibrationParameters());
        } else {
            System.out.println("Not enough data for calibration yet");
            System.out.println(trajectoryHelper.getCalibrationStats());
        }
    }
    
    /**
     * Manually update calibration parameters.
     * Use this if you calculate parameters from logs manually.
     */
    public void updateCalibration(double dragCoeff, double speedEfficiency) {
        CalibrationParameters newParams = trajectoryHelper.getCalibrationParameters()
            .withDragCoefficient(dragCoeff)
            .withSpeedEfficiency(speedEfficiency);
        
        trajectoryHelper.updateCalibration(newParams);
        
        System.out.println("Calibration updated manually");
    }
    
    /**
     * Check if target is reachable before calculating full solution.
     */
    public boolean canReachTarget(Pose2d robotPose) {
        return trajectoryHelper.getController().canReachTarget(
            robotPose.getX(),
            robotPose.getY(),
            SHOOTER_HEIGHT,
            LAUNCH_SPEED
        );
    }
    
    /**
     * Get last calculated solution.
     */
    public InverseSolver.SolutionResult getLastSolution() {
        return lastSolution;
    }
    
    /**
     * Check if we have a valid shooting solution.
     */
    public boolean hasValidSolution() {
        return lastSolution != null && lastSolution.isHit();
    }
    
    @Override
    public void periodic() {
        // Calibration parameters are automatically logged by the helper
        // No additional logging needed!
    }
}
