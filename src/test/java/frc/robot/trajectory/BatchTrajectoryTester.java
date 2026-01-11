package frc.robot.trajectory;

import frc.robot.trajectory.util.Vector3D;
import frc.robot.trajectory.solver.InverseSolver;
import frc.robot.trajectory.simulation.TrajectorySimulator;
import frc.robot.trajectory.physics.PhysicsModel;
import frc.robot.trajectory.physics.PhysicsConstants;
import frc.robot.trajectory.physics.ProjectileProperties;
import frc.robot.trajectory.physics.HubGeometry;
import frc.robot.trajectory.calibration.CalibrationParameters;
import frc.robot.trajectory.solver.RK4Integrator;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Batch trajectory tester that reads target coordinates from a file
 * and computes optimal shooting solutions for each target.
 */
public class BatchTrajectoryTester {
    
    private final InverseSolver solver;
    private final double defaultSpeed;
    private final double defaultSpinRate;
    
    public BatchTrajectoryTester() {
        this(12.0, 200.0);
    }
    
    public BatchTrajectoryTester(double defaultSpeed, double defaultSpinRate) {
        CalibrationParameters calibration = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel physics = new PhysicsModel(calibration, projectile);
        HubGeometry hub = new HubGeometry();
        TrajectorySimulator simulator = new TrajectorySimulator(physics, hub);
        this.solver = new InverseSolver(simulator);
        this.defaultSpeed = defaultSpeed;
        this.defaultSpinRate = defaultSpinRate;
    }
    
    /**
     * Read target coordinates from file.
     * Format: x,y,z per line, # for comments
     */
    public List<Vector3D> readTargetsFromFile(String filename) throws IOException {
        List<Vector3D> targets = new ArrayList<>();
        
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            int lineNum = 0;
            
            while ((line = reader.readLine()) != null) {
                lineNum++;
                line = line.trim();
                
                // Skip empty lines and comments
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                
                try {
                    String[] parts = line.split(",");
                    if (parts.length != 3) {
                        System.err.println("Line " + lineNum + ": Expected 3 values (x,y,z), got " + parts.length);
                        continue;
                    }
                    
                    double x = Double.parseDouble(parts[0].trim());
                    double y = Double.parseDouble(parts[1].trim());
                    double z = Double.parseDouble(parts[2].trim());
                    
                    targets.add(new Vector3D(x, y, z));
                    
                } catch (NumberFormatException e) {
                    System.err.println("Line " + lineNum + ": Invalid number format - " + e.getMessage());
                }
            }
        }
        
        return targets;
    }
    
    /**
     * Compute and display optimal solution for a target.
     */
    public void testTarget(Vector3D target, double speed, double spinRate) {
        Vector3D spin = new Vector3D(0, spinRate, 0); // Backspin
        
        InverseSolver.SolutionResult solution = solver.solve(target, speed, spin);
        
        double distance = Math.sqrt(target.x * target.x + target.y * target.y);
        
        System.out.println("Target: (" + 
            String.format("%.2f", target.x) + ", " +
            String.format("%.2f", target.y) + ", " +
            String.format("%.2f", target.z) + ") m");
        System.out.println("  Distance: " + String.format("%.2f", distance) + " m");
        
        if (solution != null) {
            System.out.println("  Launch Yaw:   " + String.format("%.2f", solution.launchYawDeg) + " deg");
            System.out.println("  Launch Pitch: " + String.format("%.2f", solution.launchPitchDeg) + " deg");
            System.out.println("  Entry Score:  " + String.format("%.4f", solution.score));
            System.out.println("  Hit: " + solution.isHit());
            
            if (solution.trajectory != null && solution.trajectory.entryState != null) {
                double vz = solution.trajectory.entryState.velocity.z;
                double speed_entry = solution.trajectory.entryState.velocity.magnitude();
                double entryAngle = 0.0;
                if (speed_entry > 0.1) {
                    entryAngle = Math.abs(Math.toDegrees(Math.asin(-vz / speed_entry)));
                }
                System.out.println("  Entry Angle:  " + String.format("%.2f", entryAngle) + " deg");
            }
        } else {
            System.out.println("  NO SOLUTION FOUND");
            System.out.println("  (Target may be out of range or unreachable)");
        }
        
        System.out.println();
    }
    
    /**
     * Process all targets from a file.
     */
    public void processFile(String filename) {
        System.out.println("==========================================");
        System.out.println("Batch Trajectory Tester");
        System.out.println("==========================================");
        System.out.println("Input file: " + filename);
        System.out.println("Default speed: " + defaultSpeed + " m/s");
        System.out.println("Default spin: " + defaultSpinRate + " rad/s");
        System.out.println("==========================================\n");
        
        List<Vector3D> targets;
        try {
            targets = readTargetsFromFile(filename);
        } catch (IOException e) {
            System.err.println("Error reading file: " + e.getMessage());
            return;
        }
        
        if (targets.isEmpty()) {
            System.out.println("No valid targets found in file.");
            return;
        }
        
        System.out.println("Processing " + targets.size() + " targets...\n");
        
        int successful = 0;
        for (int i = 0; i < targets.size(); i++) {
            System.out.println("Target " + (i + 1) + " of " + targets.size() + ":");
            Vector3D target = targets.get(i);
            testTarget(target, defaultSpeed, defaultSpinRate);
            
            InverseSolver.SolutionResult solution = solver.solve(target, defaultSpeed, new Vector3D(0, defaultSpinRate, 0));
            if (solution != null && solution.isHit()) {
                successful++;
            }
        }
        
        System.out.println("==========================================");
        System.out.println("Summary:");
        System.out.println("  Total targets: " + targets.size());
        System.out.println("  Successful:    " + successful);
        System.out.println("  Failed:        " + (targets.size() - successful));
        System.out.println("  Success rate:  " + 
            String.format("%.1f", 100.0 * successful / targets.size()) + "%");
        System.out.println("==========================================");
    }
    
    /**
     * Main method for standalone execution.
     */
    public static void main(String[] args) {
        String filename = args.length > 0 ? args[0] : "test-targets.txt";
        double speed = args.length > 1 ? Double.parseDouble(args[1]) : 12.0;
        double spin = args.length > 2 ? Double.parseDouble(args[2]) : 200.0;
        
        BatchTrajectoryTester tester = new BatchTrajectoryTester(speed, spin);
        tester.processFile(filename);
    }
}
