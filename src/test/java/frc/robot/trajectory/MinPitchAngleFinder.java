package frc.robot.trajectory;

import frc.robot.trajectory.util.Vector3D;
import frc.robot.trajectory.solver.InverseSolver;
import frc.robot.trajectory.simulation.TrajectorySimulator;
import frc.robot.trajectory.simulation.SpinShotSimulator;
import frc.robot.trajectory.physics.PhysicsModel;
import frc.robot.trajectory.physics.ProjectileProperties;
import frc.robot.trajectory.physics.HubGeometry;
import frc.robot.trajectory.physics.PhysicsConstants;
import frc.robot.trajectory.calibration.CalibrationParameters;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Systematically tests different minimum pitch angles to find the highest
 * minimum that still allows 100% success rate on all test scenarios.
 */
public class MinPitchAngleFinder {
    
    private final TrajectorySimulator trajSimulator;
    private final double maxBallSpeedMS;
    private final double defaultSpinRate;
    
    public MinPitchAngleFinder(double maxBallSpeedFPS, double defaultSpinRate) {
        double maxBallSpeedMS = maxBallSpeedFPS * 0.3048;
        CalibrationParameters calibration = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel physics = new PhysicsModel(calibration, projectile);
        HubGeometry hub = new HubGeometry();
        this.trajSimulator = new TrajectorySimulator(physics, hub);
        this.maxBallSpeedMS = maxBallSpeedMS;
        this.defaultSpinRate = defaultSpinRate;
    }
    
    /**
     * Test a specific minimum pitch angle across all test scenarios.
     * Returns the number of successful shots.
     */
    public TestResult testMinimumPitchAngle(double minPitchDeg, String filename) {
        TestResult result = new TestResult(minPitchDeg);
        
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            Vector3D robotPos = null;
            double turretYaw = 0;
            double turretPitch = 0;
            double spinRate = defaultSpinRate;
            int scenarioCount = 0;
            
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                
                if (line.startsWith("ROBOT_XY:")) {
                    String[] parts = line.substring(9).trim().split(",");
                    double x = Double.parseDouble(parts[0].trim());
                    double y = Double.parseDouble(parts[1].trim());
                    robotPos = new Vector3D(x, y, 0.5); // Shooter at 0.5m height
                    
                } else if (line.startsWith("CURRENT_TURRET_DEGREE:")) {
                    turretYaw = Double.parseDouble(line.substring(22).trim());
                    
                } else if (line.startsWith("CURRENT_HOOD_DEGREE:")) {
                    turretPitch = Double.parseDouble(line.substring(20).trim());
                    
                } else if (line.startsWith("LAUNCH_PARAMS:")) {
                    String[] parts = line.substring(14).trim().split(",");
                    if (parts.length >= 2) {
                        spinRate = Double.parseDouble(parts[1].trim());
                    }
                    
                } else if (line.equals("---")) {
                    // End of scenario
                    if (robotPos != null) {
                        scenarioCount++;
                        result.totalScenarios++;
                        
                        // Test this scenario with the given minimum pitch
                        boolean success = testScenario(robotPos, turretYaw, turretPitch, 
                                                      spinRate, minPitchDeg);
                        if (success) {
                            result.successCount++;
                        } else {
                            result.failedScenarios.add(scenarioCount);
                        }
                    }
                    
                    // Reset for next scenario
                    robotPos = null;
                    turretYaw = 0;
                    turretPitch = 0;
                    spinRate = defaultSpinRate;
                }
            }
        } catch (IOException e) {
            System.err.println("Error reading file: " + e.getMessage());
        }
        
        return result;
    }
    
    /**
     * Test a single scenario with the given minimum pitch angle.
     */
    private boolean testScenario(Vector3D robotPos, double turretYaw, double turretPitch,
                                 double spinRate, double minPitchDeg) {
        Vector3D targetPos = new Vector3D(0, 0, 1.83); // Hub center
        double dx = targetPos.x - robotPos.x;
        double dy = targetPos.y - robotPos.y;
        double dz = targetPos.z - robotPos.z;
        
        Vector3D spin = new Vector3D(0, spinRate, 0); // Backspin
        
        double maxSpeed = maxBallSpeedMS;
        double targetYawDeg = Math.toDegrees(Math.atan2(dy, dx));
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        double geometricPitch = Math.toDegrees(Math.atan2(dz, distanceToTarget));
        
        // Determine speed range
        double minSpeed;
        if (distanceToTarget < 0.3) {
            minSpeed = 1.0;
        } else if (distanceToTarget < 0.6) {
            minSpeed = Math.max(1.2, distanceToTarget * 2.2);
        } else if (distanceToTarget < 1.0) {
            minSpeed = Math.max(1.5, distanceToTarget * 2.0);
        } else if (distanceToTarget < 1.5) {
            minSpeed = Math.max(2.5, distanceToTarget * 2.0);
        } else if (distanceToTarget > 4.27) {
            minSpeed = Math.max(7.5, distanceToTarget * 1.5);
        } else if (distanceToTarget > 3.66) {
            minSpeed = Math.max(7.0, distanceToTarget * 1.8);
        } else if (distanceToTarget > 3.048) {
            minSpeed = Math.max(6.5, distanceToTarget * 2.0);
        } else if (distanceToTarget > 2.4) {
            minSpeed = Math.max(6.0, distanceToTarget * 2.0);
        } else {
            minSpeed = Math.max(4.5, distanceToTarget * 2.5);
        }
        minSpeed = Math.min(minSpeed, maxSpeed);
        
        // Search for a solution
        double bestScore = -Double.MAX_VALUE;
        boolean foundSolution = false;
        
        // Test pitch angles around geometric estimate
        for (double pitchOffset = -16.0; pitchOffset <= 16.0; pitchOffset += 4.0) {
            double testPitch = geometricPitch + pitchOffset;
            
            // Apply the minimum pitch constraint we're testing
            double maxAllowedPitch = (distanceToTarget < 1.0) ? 89.0 : 85.0;
            if (testPitch < minPitchDeg || testPitch > maxAllowedPitch) {
                continue;
            }
            
            // Test a few speeds
            for (int s = 0; s <= 5; s++) {
                double speedStep = (maxSpeed - minSpeed) / 5.0;
                double testSpeed = minSpeed + s * speedStep;
                
                // Test a few yaw offsets
                for (double yawOffset = -8.0; yawOffset <= 8.0; yawOffset += 4.0) {
                    double testYaw = targetYawDeg + yawOffset;
                    
                    TrajectorySimulator.TrajectoryResult result = 
                        trajSimulator.simulateWithShooterModel(
                            robotPos, testSpeed, testYaw, testPitch, spin);
                    
                    if (result.hitTarget) {
                        double score = result.entryScore;
                        if (score > bestScore) {
                            bestScore = score;
                            foundSolution = true;
                        }
                    }
                }
            }
        }
        
        return foundSolution;
    }
    
    /**
     * Result of testing a specific minimum pitch angle.
     */
    public static class TestResult {
        public double minPitchDeg;
        public int totalScenarios = 0;
        public int successCount = 0;
        public List<Integer> failedScenarios = new ArrayList<>();
        
        public TestResult(double minPitchDeg) {
            this.minPitchDeg = minPitchDeg;
        }
        
        public double getSuccessRate() {
            return totalScenarios > 0 ? (double) successCount / totalScenarios : 0.0;
        }
        
        @Override
        public String toString() {
            return String.format("Min Pitch: %.1f° - Success: %d/%d (%.1f%%) - Failed: %s",
                minPitchDeg, successCount, totalScenarios, getSuccessRate() * 100,
                failedScenarios.isEmpty() ? "none" : failedScenarios.toString());
        }
    }
    
    public static void main(String[] args) {
        String filename = args.length > 0 ? args[0] : "test-targets.txt";
        double maxSpeedFPS = args.length > 1 ? Double.parseDouble(args[1]) : 30.0;
        double spinRate = args.length > 2 ? Double.parseDouble(args[2]) : 200.0;
        
        MinPitchAngleFinder finder = new MinPitchAngleFinder(maxSpeedFPS, spinRate);
        
        System.out.println("========================================");
        System.out.println("MINIMUM PITCH ANGLE FINDER");
        System.out.println("========================================");
        System.out.println("Max Ball Speed: " + maxSpeedFPS + " ft/s");
        System.out.println("Spin Rate: " + spinRate + " rad/s");
        System.out.println("Test File: " + filename);
        System.out.println();
        System.out.println("Testing different minimum pitch angles...");
        System.out.println("----------------------------------------");
        System.out.println();
        
        // Test minimum pitch angles from 0° to 30° in 5° increments
        double[] testAngles = {0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0};
        
        TestResult lastFullSuccess = null;
        
        for (double minPitch : testAngles) {
            System.out.println("Testing minimum pitch: " + minPitch + "°...");
            TestResult result = finder.testMinimumPitchAngle(minPitch, filename);
            System.out.println(result);
            System.out.println();
            
            if (result.getSuccessRate() >= 1.0) {
                lastFullSuccess = result;
            } else {
                // Once we start failing, we found the limit
                System.out.println("----------------------------------------");
                System.out.println("RESULT: Maximum minimum pitch angle is " + 
                                 (lastFullSuccess != null ? lastFullSuccess.minPitchDeg : "less than 0") + 
                                 "°");
                System.out.println("========================================");
                return;
            }
        }
        
        // If we get here, even 30° works
        System.out.println("----------------------------------------");
        System.out.println("RESULT: Minimum pitch angle can be at least 30° or higher");
        System.out.println("(May need to test higher values)");
        System.out.println("========================================");
    }
}
