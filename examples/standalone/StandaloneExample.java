package examples.standalone;

import com.ruthiefrc.trajectory.*;

/**
 * Standalone example showing how to use the trajectory calculator
 * without any specific robot framework.
 * 
 * This works with any Java robot control system - just provide position
 * data and use the calculated angles.
 */
public class StandaloneExample {
    
    private final ShooterController shooter;
    
    public StandaloneExample() {
        // Create with default parameters
        shooter = new ShooterController();
        
        // Or with custom calibration
        // CalibrationParameters params = new CalibrationParameters()
        //     .withDragCoefficient(0.45)
        //     .withSpeedEfficiency(0.92);
        // shooter = new ShooterController(params);
    }
    
    /**
     * Calculate and execute a shot from the robot's current position.
     */
    public void calculateAndShoot(double robotX, double robotY, double shooterHeight) {
        // Define launch parameters
        double launchSpeed = 12.0;  // m/s
        double spinRate = 100.0;     // rad/s
        
        // Calculate optimal launch angles
        InverseSolver.SolutionResult solution = shooter.calculateShot(
            robotX, robotY, shooterHeight, launchSpeed, spinRate
        );
        
        // Check if we have a valid solution
        if (solution.isHit()) {
            System.out.println("Valid shooting solution found!");
            System.out.println("  Yaw angle: " + solution.launchYawDeg + "°");
            System.out.println("  Pitch angle: " + solution.launchPitchDeg + "°");
            System.out.println("  Quality score: " + solution.score);
            
            // Apply to your robot's mechanisms
            setTurretAngle(solution.launchYawDeg);
            setHoodAngle(solution.launchPitchDeg);
            setFlywheelSpeed(launchSpeed);
            
            // Wait for mechanisms to reach position
            waitForReady();
            
            // Fire!
            fire();
            
            // Log the result for calibration
            boolean hit = checkIfHit();  // Your detection logic
            shooter.logShot(robotX, robotY, shooterHeight, solution, 
                          launchSpeed, spinRate, hit);
            
        } else {
            System.out.println("No valid solution - target not reachable");
            System.out.println("  Score: " + solution.score);
        }
    }
    
    /**
     * Quick check if target is reachable before full calculation.
     */
    public boolean isTargetReachable(double robotX, double robotY, double shooterHeight) {
        double maxSpeed = 15.0;  // Your maximum flywheel speed
        return shooter.canReachTarget(robotX, robotY, shooterHeight, maxSpeed);
    }
    
    /**
     * Test a specific trajectory (useful for debugging).
     */
    public void testTrajectory(double robotX, double robotY, double shooterHeight,
                               double yawDeg, double pitchDeg) {
        TrajectorySimulator.TrajectoryResult result = shooter.simulateTrajectory(
            robotX, robotY, shooterHeight,
            12.0,      // launch speed
            yawDeg,    // yaw angle
            pitchDeg,  // pitch angle
            100.0      // spin rate
        );
        
        System.out.println("Trajectory simulation:");
        System.out.println("  Hit target: " + result.hitTarget);
        System.out.println("  Max height: " + result.getMaxHeight() + " m");
        System.out.println("  Flight time: " + result.getFlightTime() + " s");
        System.out.println("  Number of points: " + result.trajectory.size());
    }
    
    /**
     * Perform calibration after collecting shot data.
     */
    public void calibrateFromData() {
        System.out.println("Calibration status: " + shooter.getCalibrationStats());
        
        if (shooter.calibrate()) {
            System.out.println("Calibration updated!");
            System.out.println("New parameters: " + 
                shooter.getCalibrationParameters());
        } else {
            System.out.println("Not enough data for calibration yet");
        }
    }
    
    // Placeholder methods - implement these for your robot
    private void setTurretAngle(double degrees) {
        // Your turret control code
        System.out.println("Setting turret to " + degrees + "°");
    }
    
    private void setHoodAngle(double degrees) {
        // Your hood control code
        System.out.println("Setting hood to " + degrees + "°");
    }
    
    private void setFlywheelSpeed(double metersPerSecond) {
        // Your flywheel control code
        System.out.println("Setting flywheel to " + metersPerSecond + " m/s");
    }
    
    private void waitForReady() {
        // Wait for mechanisms to reach target positions
        System.out.println("Waiting for ready...");
    }
    
    private void fire() {
        // Trigger shot mechanism
        System.out.println("Firing!");
    }
    
    private boolean checkIfHit() {
        // Your hit detection logic (vision, scoring system, etc.)
        return true;  // Placeholder
    }
    
    /**
     * Example main method showing usage.
     */
    public static void main(String[] args) {
        StandaloneExample example = new StandaloneExample();
        
        // Example: Robot at position (4, 2, 0.8) meters
        double robotX = 4.0;
        double robotY = 2.0;
        double shooterHeight = 0.8;
        
        // Check if reachable
        if (example.isTargetReachable(robotX, robotY, shooterHeight)) {
            // Calculate and execute shot
            example.calculateAndShoot(robotX, robotY, shooterHeight);
        } else {
            System.out.println("Target is too far - move closer!");
        }
        
        // Test specific angles
        example.testTrajectory(robotX, robotY, shooterHeight, 26.57, 35.0);
        
        // Calibrate after collecting data
        example.calibrateFromData();
    }
}
