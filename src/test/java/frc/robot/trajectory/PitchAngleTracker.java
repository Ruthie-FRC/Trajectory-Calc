package frc.robot.trajectory;

import frc.robot.trajectory.util.Vector3D;
import frc.robot.trajectory.solver.InverseSolver;
import frc.robot.trajectory.simulation.TrajectorySimulator;
import frc.robot.trajectory.physics.PhysicsModel;
import frc.robot.trajectory.physics.ProjectileProperties;
import frc.robot.trajectory.physics.HubGeometry;
import frc.robot.trajectory.calibration.CalibrationParameters;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

/**
 * Tracks the actual pitch angles used in all successful solutions
 * to find the shallowest angle needed.
 */
public class PitchAngleTracker {
    
    public static void main(String[] args) {
        String filename = args.length > 0 ? args[0] : "test-targets.txt";
        double maxSpeedFPS = args.length > 1 ? Double.parseDouble(args[1]) : 30.0;
        double spinRate = args.length > 2 ? Double.parseDouble(args[2]) : 200.0;
        
        System.out.println("========================================");
        System.out.println("PITCH ANGLE TRACKER");
        System.out.println("========================================");
        System.out.println("Finding actual pitch angles used in solutions...");
        System.out.println();
        
        // Use minimum angle of 0 to not restrict the search
        TurretTrajectoryTester tester = new TurretTrajectoryTester(maxSpeedFPS, spinRate, 0.0);
        
        double minPitchUsed = Double.MAX_VALUE;
        double maxPitchUsed = -Double.MAX_VALUE;
        int scenarioWithMinPitch = -1;
        int totalScenarios = 0;
        int successCount = 0;
        
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            Vector3D robotPos = null;
            double turretYaw = 0;
            double turretPitch = 0;
            double testSpin = spinRate;
            int scenarioNum = 0;
            
            while ((line = reader.readLine()) != null) {
                line = line.trim();
                
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                
                if (line.startsWith("ROBOT_XY:")) {
                    String[] parts = line.substring(9).trim().split(",");
                    double x = Double.parseDouble(parts[0].trim());
                    double y = Double.parseDouble(parts[1].trim());
                    robotPos = new Vector3D(x, y, 0.5);
                    
                } else if (line.startsWith("CURRENT_TURRET_DEGREE:")) {
                    turretYaw = Double.parseDouble(line.substring(22).trim());
                    
                } else if (line.startsWith("CURRENT_HOOD_DEGREE:")) {
                    turretPitch = Double.parseDouble(line.substring(20).trim());
                    
                } else if (line.startsWith("LAUNCH_PARAMS:")) {
                    String[] parts = line.substring(14).trim().split(",");
                    if (parts.length >= 2) {
                        testSpin = Double.parseDouble(parts[1].trim());
                    }
                    
                } else if (line.equals("---")) {
                    if (robotPos != null) {
                        scenarioNum++;
                        totalScenarios++;
                        
                        TurretTrajectoryTester.TestConfig config = 
                            new TurretTrajectoryTester.TestConfig(
                                robotPos,
                                new Vector3D(0, 0, 1.83),
                                turretYaw,
                                turretPitch,
                                testSpin
                            );
                        
                        TurretTrajectoryTester.SolutionOutput solution = 
                            tester.computeOptimalShot(config);
                        
                        if (solution.hasShot) {
                            successCount++;
                            double pitchUsed = solution.finalPitch;
                            
                            if (pitchUsed < minPitchUsed) {
                                minPitchUsed = pitchUsed;
                                scenarioWithMinPitch = scenarioNum;
                            }
                            
                            if (pitchUsed > maxPitchUsed) {
                                maxPitchUsed = pitchUsed;
                            }
                            
                            // Print progress every 50 scenarios
                            if (scenarioNum % 50 == 0) {
                                System.out.println("Processed " + scenarioNum + " scenarios...");
                                System.out.println("  Current min pitch: " + String.format("%.2f°", minPitchUsed));
                            }
                        }
                    }
                    
                    robotPos = null;
                    turretYaw = 0;
                    turretPitch = 0;
                    testSpin = spinRate;
                }
            }
        } catch (IOException e) {
            System.err.println("Error reading file: " + e.getMessage());
        }
        
        System.out.println();
        System.out.println("========================================");
        System.out.println("RESULTS:");
        System.out.println("========================================");
        System.out.println("Total scenarios: " + totalScenarios);
        System.out.println("Successful: " + successCount);
        System.out.println("Failed: " + (totalScenarios - successCount));
        System.out.println();
        System.out.println("SHALLOWEST PITCH ANGLE USED: " + String.format("%.2f°", minPitchUsed));
        System.out.println("  (Found in scenario #" + scenarioWithMinPitch + ")");
        System.out.println();
        System.out.println("STEEPEST PITCH ANGLE USED: " + String.format("%.2f°", maxPitchUsed));
        System.out.println();
        System.out.println("RECOMMENDATION:");
        System.out.println("  Minimum pitch angle can be set to: " + String.format("%.0f°", Math.floor(minPitchUsed)));
        System.out.println("  (Rounding down from " + String.format("%.2f°", minPitchUsed) + " to ensure all scenarios work)");
        System.out.println("========================================");
    }
}
