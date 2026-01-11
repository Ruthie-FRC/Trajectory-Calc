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
 * Enhanced trajectory tester that accounts for robot position and turret orientation.
 * Tests shots with configurable turret yaw/pitch offsets and provides comprehensive
 * solution output including success probability.
 */
public class TurretTrajectoryTester {
    
    private final InverseSolver solver;
    private final SpinShotSimulator simulator;
    private final double defaultSpeed;
    private final double defaultSpinRate;
    
    /**
     * Configuration for a test scenario.
     */
    public static class TestConfig {
        public Vector3D robotPosition;      // Robot position on field (x, y, z)
        public Vector3D targetPosition;     // Target position (x, y, z)
        public double turretYawOffset;      // Turret yaw offset from robot heading (degrees)
        public double turretPitchOffset;    // Turret pitch offset from horizontal (degrees)
        public double speed;                 // Launch speed (m/s)
        public double spinRate;              // Ball spin rate (rad/s)
        
        public TestConfig(Vector3D robotPos, Vector3D targetPos, 
                          double yawOffset, double pitchOffset,
                          double speed, double spinRate) {
            this.robotPosition = robotPos;
            this.targetPosition = targetPos;
            this.turretYawOffset = yawOffset;
            this.turretPitchOffset = pitchOffset;
            this.speed = speed;
            this.spinRate = spinRate;
        }
    }
    
    /**
     * Detailed solution output.
     */
    public static class SolutionOutput {
        public boolean hasShot;
        public double requiredPitchAdjust;   // Pitch adjustment needed (degrees)
        public double requiredYawAdjust;     // Yaw adjustment needed (degrees)
        public double finalPitch;            // Final absolute pitch angle (degrees)
        public double finalYaw;              // Final absolute yaw angle (degrees)
        public double shooterRPM;            // Shooter motor RPM
        public double spinRate;              // Ball spin rate (rad/s)
        public Vector3D spinAxis;            // Spin axis vector
        public double successProbability;    // Success probability (0-1)
        public double riskScore;             // Bounce-out risk score (lower is better)
        public double entryAngle;            // Entry angle at target (degrees)
        public double flightTime;            // Time to target (seconds)
        public double apexHeight;            // Maximum height (meters)
        public String trajectory;            // Brief trajectory description
        
        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            if (!hasShot) {
                sb.append("  NO VIABLE SHOT FOUND\n");
                sb.append("  Target may be out of range or blocked.\n");
                return sb.toString();
            }
            
            sb.append("  OPTIMAL SOLUTION:\n");
            sb.append(String.format("    Pitch Adjustment:  %+.2f° (Final: %.2f°)\n", 
                requiredPitchAdjust, finalPitch));
            sb.append(String.format("    Yaw Adjustment:    %+.2f° (Final: %.2f°)\n", 
                requiredYawAdjust, finalYaw));
            sb.append(String.format("    Shooter Motor:     %.0f RPM\n", shooterRPM));
            sb.append(String.format("    Ball Spin:         %.1f rad/s (%.0f RPM)\n", 
                spinRate, spinRate * 60 / (2 * Math.PI)));
            sb.append(String.format("    Success Rate:      %.1f%%\n", successProbability * 100));
            sb.append(String.format("    Risk Score:        %.4f (lower is better)\n", riskScore));
            sb.append(String.format("    Entry Angle:       %.2f°\n", entryAngle));
            sb.append(String.format("    Flight Time:       %.2f s\n", flightTime));
            sb.append(String.format("    Apex Height:       %.2f m\n", apexHeight));
            sb.append(String.format("    Trajectory:        %s\n", trajectory));
            
            return sb.toString();
        }
    }
    
    public TurretTrajectoryTester() {
        this(12.0, 200.0);
    }
    
    public TurretTrajectoryTester(double defaultSpeed, double defaultSpinRate) {
        CalibrationParameters calibration = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel physics = new PhysicsModel(calibration, projectile);
        HubGeometry hub = new HubGeometry();
        TrajectorySimulator trajSim = new TrajectorySimulator(physics, hub);
        this.solver = new InverseSolver(trajSim);
        this.simulator = new SpinShotSimulator(calibration, projectile);
        this.defaultSpeed = defaultSpeed;
        this.defaultSpinRate = defaultSpinRate;
    }
    
    /**
     * Compute optimal solution accounting for turret position and orientation.
     */
    public SolutionOutput computeOptimalShot(TestConfig config) {
        SolutionOutput output = new SolutionOutput();
        
        // Calculate relative position from turret to target
        double dx = config.targetPosition.x - config.robotPosition.x;
        double dy = config.targetPosition.y - config.robotPosition.y;
        double dz = config.targetPosition.z - config.robotPosition.z;
        
        Vector3D relativeTarget = new Vector3D(dx, dy, dz);
        Vector3D spin = new Vector3D(0, config.spinRate, 0); // Backspin
        
        // Solve for optimal trajectory from turret position
        InverseSolver.SolutionResult solution = solver.solve(
            relativeTarget, config.speed, spin);
        
        if (solution == null || !solution.isHit()) {
            output.hasShot = false;
            return output;
        }
        
        output.hasShot = true;
        
        // Calculate adjustments needed from current turret orientation
        output.requiredYawAdjust = solution.launchYawDeg - config.turretYawOffset;
        output.requiredPitchAdjust = solution.launchPitchDeg - config.turretPitchOffset;
        output.finalYaw = solution.launchYawDeg;
        output.finalPitch = solution.launchPitchDeg;
        
        // Convert speed to typical FRC shooter RPM (assuming 4" wheel diameter)
        double wheelDiameterMeters = 0.1016; // 4 inches
        double wheelCircumference = Math.PI * wheelDiameterMeters;
        output.shooterRPM = (config.speed / wheelCircumference) * 60.0;
        
        output.spinRate = config.spinRate;
        output.spinAxis = spin.normalize();
        
        // Calculate success probability from risk score
        // Lower risk = higher probability
        output.riskScore = solution.score;
        output.successProbability = Math.max(0.0, Math.min(1.0, 1.0 - solution.score));
        
        // Extract trajectory details
        if (solution.trajectory != null) {
            if (solution.trajectory.entryState != null) {
                double vz = solution.trajectory.entryState.velocity.z;
                double speedAtEntry = solution.trajectory.entryState.velocity.magnitude();
                if (speedAtEntry > 0.1) {
                    output.entryAngle = Math.abs(Math.toDegrees(Math.asin(-vz / speedAtEntry)));
                }
            }
            
            output.flightTime = solution.trajectory.timeToTarget;
            output.apexHeight = solution.trajectory.apexHeight;
            
            // Generate trajectory description
            double distance = Math.sqrt(dx * dx + dy * dy);
            if (distance < 3.5) {
                output.trajectory = "Close-range, high-arc shot";
            } else if (distance < 6.0) {
                output.trajectory = "Medium-range trajectory";
            } else {
                output.trajectory = "Long-range shot";
            }
            
            if (Math.abs(dy) > 1.0) {
                output.trajectory += ", significant lateral component";
            }
        }
        
        return output;
    }
    
    /**
     * Read test configurations from file.
     * New format uses labeled sections:
     *   ROBOT_POSITION: x, y, z
     *   TARGET_POSITION: x, y, z
     *   TURRET_OFFSET: yaw, pitch
     *   LAUNCH_PARAMS: speed, spin (optional)
     *   ---
     */
    public List<TestConfig> readConfigsFromFile(String filename) throws IOException {
        List<TestConfig> configs = new ArrayList<>();
        
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            int lineNum = 0;
            
            Vector3D robotPos = null;
            Vector3D targetPos = null;
            Double yawOffset = null;
            Double pitchOffset = null;
            Double speed = null;
            Double spin = null;
            
            while ((line = reader.readLine()) != null) {
                lineNum++;
                line = line.trim();
                
                // Skip empty lines and comments
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }
                
                // Check for section separator
                if (line.equals("---")) {
                    // Complete scenario - validate and add
                    if (robotPos != null && targetPos != null && 
                        yawOffset != null && pitchOffset != null) {
                        
                        double finalSpeed = (speed != null) ? speed : defaultSpeed;
                        double finalSpin = (spin != null) ? spin : defaultSpinRate;
                        
                        configs.add(new TestConfig(
                            robotPos, targetPos, yawOffset, pitchOffset, finalSpeed, finalSpin));
                    } else {
                        System.err.println("Line " + lineNum + 
                            ": Incomplete scenario (missing required fields)");
                    }
                    
                    // Reset for next scenario
                    robotPos = null;
                    targetPos = null;
                    yawOffset = null;
                    pitchOffset = null;
                    speed = null;
                    spin = null;
                    continue;
                }
                
                // Parse labeled lines
                try {
                    if (line.startsWith("ROBOT_POSITION:")) {
                        String values = line.substring("ROBOT_POSITION:".length()).trim();
                        String[] parts = values.split(",");
                        if (parts.length == 3) {
                            robotPos = new Vector3D(
                                Double.parseDouble(parts[0].trim()),
                                Double.parseDouble(parts[1].trim()),
                                Double.parseDouble(parts[2].trim())
                            );
                        }
                    } else if (line.startsWith("TARGET_POSITION:")) {
                        String values = line.substring("TARGET_POSITION:".length()).trim();
                        String[] parts = values.split(",");
                        if (parts.length == 3) {
                            targetPos = new Vector3D(
                                Double.parseDouble(parts[0].trim()),
                                Double.parseDouble(parts[1].trim()),
                                Double.parseDouble(parts[2].trim())
                            );
                        }
                    } else if (line.startsWith("TURRET_OFFSET:")) {
                        String values = line.substring("TURRET_OFFSET:".length()).trim();
                        String[] parts = values.split(",");
                        if (parts.length == 2) {
                            yawOffset = Double.parseDouble(parts[0].trim());
                            pitchOffset = Double.parseDouble(parts[1].trim());
                        }
                    } else if (line.startsWith("LAUNCH_PARAMS:")) {
                        String values = line.substring("LAUNCH_PARAMS:".length()).trim();
                        String[] parts = values.split(",");
                        if (parts.length == 2) {
                            speed = Double.parseDouble(parts[0].trim());
                            spin = Double.parseDouble(parts[1].trim());
                        }
                    }
                } catch (NumberFormatException e) {
                    System.err.println("Line " + lineNum + 
                        ": Invalid number format - " + e.getMessage());
                }
            }
        }
        
        return configs;
    }
    
    /**
     * Process all test configurations from a file.
     */
    public void processFile(String filename) {
        System.out.println("=======================================================");
        System.out.println("Turret Trajectory Tester");
        System.out.println("=======================================================");
        System.out.println("Input file: " + filename);
        System.out.println("Default speed: " + defaultSpeed + " m/s");
        System.out.println("Default spin: " + defaultSpinRate + " rad/s");
        System.out.println("=======================================================\n");
        
        List<TestConfig> configs;
        try {
            configs = readConfigsFromFile(filename);
        } catch (IOException e) {
            System.err.println("Error reading file: " + e.getMessage());
            return;
        }
        
        if (configs.isEmpty()) {
            System.out.println("No valid configurations found in file.");
            return;
        }
        
        System.out.println("Processing " + configs.size() + " test scenarios...\n");
        
        int successful = 0;
        double totalSuccessProb = 0.0;
        
        for (int i = 0; i < configs.size(); i++) {
            TestConfig config = configs.get(i);
            System.out.println("Scenario " + (i + 1) + " of " + configs.size() + ":");
            System.out.println("  Robot at: (" + 
                String.format("%.2f", config.robotPosition.x) + ", " +
                String.format("%.2f", config.robotPosition.y) + ", " +
                String.format("%.2f", config.robotPosition.z) + ") m");
            System.out.println("  Target at: (" + 
                String.format("%.2f", config.targetPosition.x) + ", " +
                String.format("%.2f", config.targetPosition.y) + ", " +
                String.format("%.2f", config.targetPosition.z) + ") m");
            System.out.println("  Current turret: Yaw " + 
                String.format("%+.1f", config.turretYawOffset) + "°, Pitch " +
                String.format("%+.1f", config.turretPitchOffset) + "°");
            System.out.println("  Launch speed: " + 
                String.format("%.2f", config.speed) + " m/s");
            
            double dx = config.targetPosition.x - config.robotPosition.x;
            double dy = config.targetPosition.y - config.robotPosition.y;
            double distance = Math.sqrt(dx * dx + dy * dy);
            System.out.println("  Distance to target: " + 
                String.format("%.2f", distance) + " m\n");
            
            SolutionOutput solution = computeOptimalShot(config);
            System.out.println(solution);
            
            if (solution.hasShot) {
                successful++;
                totalSuccessProb += solution.successProbability;
            }
            
            System.out.println("-------------------------------------------------------\n");
        }
        
        double avgSuccessProb = successful > 0 ? totalSuccessProb / successful : 0.0;
        
        System.out.println("=======================================================");
        System.out.println("SUMMARY:");
        System.out.println("  Total scenarios:       " + configs.size());
        System.out.println("  Viable shots found:    " + successful);
        System.out.println("  No solution:           " + (configs.size() - successful));
        System.out.println("  Avg success prob:      " + 
            String.format("%.1f%%", avgSuccessProb * 100));
        System.out.println("=======================================================");
    }
    
    /**
     * Main method for standalone execution.
     */
    public static void main(String[] args) {
        String filename = args.length > 0 ? args[0] : "turret-test-configs.txt";
        double speed = args.length > 1 ? Double.parseDouble(args[1]) : 12.0;
        double spin = args.length > 2 ? Double.parseDouble(args[2]) : 200.0;
        
        TurretTrajectoryTester tester = new TurretTrajectoryTester(speed, spin);
        tester.processFile(filename);
    }
}
