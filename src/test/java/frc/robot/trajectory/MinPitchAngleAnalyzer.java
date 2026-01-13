package frc.robot.trajectory;

import frc.robot.trajectory.util.Vector3D;
import frc.robot.trajectory.solver.InverseSolver;
import frc.robot.trajectory.simulation.TrajectorySimulator;
import frc.robot.trajectory.simulation.SpinShotSimulator;
import frc.robot.trajectory.physics.PhysicsModel;
import frc.robot.trajectory.physics.ProjectileProperties;
import frc.robot.trajectory.physics.HubGeometry;
import frc.robot.trajectory.calibration.CalibrationParameters;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Analyzer to find the actual minimum pitch angle used across all test scenarios.
 */
public class MinPitchAngleAnalyzer {
    
    public static void main(String[] args) {
        String filename = args.length > 0 ? args[0] : "test-targets.txt";
        double maxSpeedFPS = args.length > 1 ? Double.parseDouble(args[1]) : 30.0;
        double spinRate = args.length > 2 ? Double.parseDouble(args[2]) : 200.0;
        
        System.out.println("========================================");
        System.out.println("MINIMUM PITCH ANGLE ANALYZER");
        System.out.println("========================================");
        System.out.println("Analyzing all 750 test scenarios to find shallowest angle used...");
        System.out.println();
        
        // Test with increasingly higher minimum angles until we start failing
        double[] testMinAngles = {0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0};
        
        for (double minAngle : testMinAngles) {
            System.out.println("Testing with minimum pitch angle: " + minAngle + "°");
            TurretTrajectoryTester tester = new TurretTrajectoryTester(maxSpeedFPS, spinRate, minAngle);
            
            int totalScenarios = 0;
            int successCount = 0;
            
            try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
                String line;
                Vector3D robotPos = null;
                double turretYaw = 0;
                double turretPitch = 0;
                double testSpin = spinRate;
                
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
            
            double successRate = totalScenarios > 0 ? 
                (double) successCount / totalScenarios * 100 : 0;
            
            System.out.println("  Total scenarios: " + totalScenarios);
            System.out.println("  Successful: " + successCount);
            System.out.println("  Failed: " + (totalScenarios - successCount));
            System.out.println("  Success rate: " + String.format("%.1f%%", successRate));
            System.out.println();
            
            if (successRate < 100.0) {
                System.out.println("========================================");
                System.out.println("RESULT:");
                System.out.println("The minimum pitch angle that maintains 100% success is between");
                if (minAngle > 0) {
                    System.out.println((minAngle - 5.0) + "° and " + minAngle + "°");
                } else {
                    System.out.println("less than " + minAngle + "°");
                }
                System.out.println("========================================");
                break;
            } else if (minAngle == testMinAngles[testMinAngles.length - 1]) {
                System.out.println("========================================");
                System.out.println("RESULT:");
                System.out.println("The minimum pitch angle can be at least " + minAngle + "°");
                System.out.println("(100% success rate maintained)");
                System.out.println("========================================");
            }
        }
    }
}
