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
 * Enhanced trajectory tester that accounts for robot position and turret orientation.
 * Tests shots with configurable turret yaw/pitch offsets and provides comprehensive
 * solution output including success probability.
 */
public class TurretTrajectoryTester {
    
    private final InverseSolver solver;
    private final SpinShotSimulator simulator;
    private final double maxFlywheelRPM;
    private final double defaultSpinRate;
    private final double wheelDiameterMeters = 0.1016; // 4 inches
    
    // Speed search heuristics
    private static final double MIN_SPEED_FACTOR = 0.8; // Minimum speed as fraction of distance
    private static final double MIN_SPEED_FLOOR = 8.0; // Absolute minimum speed in m/s
    private static final int SPEED_SEARCH_STEPS = 20; // Number of speeds to try
    
    /**
     * Configuration for a test scenario.
     */
    public static class TestConfig {
        public Vector3D robotPosition;      // Robot position on field (x, y, z)
        public Vector3D targetPosition;     // Target position (x, y, z)
        public double turretYawOffset;      // Turret yaw offset from robot heading (degrees)
        public double turretPitchOffset;    // Turret pitch offset from horizontal (degrees)
        public double spinRate;              // Ball spin rate (rad/s)
        
        public TestConfig(Vector3D robotPos, Vector3D targetPos, 
                          double yawOffset, double pitchOffset,
                          double spinRate) {
            this.robotPosition = robotPos;
            this.targetPosition = targetPos;
            this.turretYawOffset = yawOffset;
            this.turretPitchOffset = pitchOffset;
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
        public double confidence;            // Trajectory confidence (0-1)
        public double marginOfError;         // Position margin of error (meters)
        public double riskScore;             // Bounce-out risk score (lower is better)
        public double entryAngle;            // Entry angle at target (degrees)
        public double flightTime;            // Time to target (seconds)
        public double apexHeight;            // Maximum height (meters)
        public String trajectory;            // Brief trajectory description
        public String trajectoryEquation;    // Full mathematical trajectory equation (non-simplified)
        
        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            if (!hasShot) {
                sb.append("  NO VIABLE SHOT FOUND\n");
                sb.append("  Target may be out of range or blocked.\n");
                return sb.toString();
            }
            
            sb.append("  OPTIMAL SOLUTION:\n");
            sb.append(String.format("    Turret Change:     %+.2f° → New position: %.2f°\n", 
                requiredYawAdjust, finalYaw));
            sb.append(String.format("    Hood Change:       %+.2f° → New position: %.2f°\n", 
                requiredPitchAdjust, finalPitch));
            sb.append("\n");
            sb.append(String.format("    Shooter Motor:     %.0f RPM (4\" wheels)\n", shooterRPM));
            
            // Determine spin type
            String spinType = "Backspin"; // Default
            if (spinAxis != null) {
                if (Math.abs(spinAxis.y) > 0.9) {
                    spinType = "Backspin";
                } else if (Math.abs(spinAxis.x) > 0.5) {
                    spinType = "Sidespin";
                } else if (spinAxis.y < -0.5) {
                    spinType = "Topspin";
                }
            }
            sb.append(String.format("    Spin Type:         %s\n", spinType));
            sb.append(String.format("    Ball Spin:         %.1f rad/s (%.0f RPM)\n", 
                spinRate, spinRate * 60 / (2 * Math.PI)));
            sb.append("\n");
            sb.append(String.format("    Success Rate:      %.1f%%\n", successProbability * 100));
            sb.append(String.format("    Confidence:        %.1f%%\n", confidence * 100));
            sb.append(String.format("    Margin of Error:   ±%.3f m\n", marginOfError));
            sb.append(String.format("    Risk Score:        %.4f (lower is better)\n", riskScore));
            sb.append(String.format("    Entry Angle:       %.2f°\n", entryAngle));
            sb.append(String.format("    Flight Time:       %.2f s\n", flightTime));
            sb.append(String.format("    Apex Height:       %.2f m\n", apexHeight));
            sb.append("\n");
            sb.append(String.format("    Trajectory:        %s\n", trajectory));
            sb.append("\n");
            sb.append("    Trajectory Equation (Full Physics Model):\n");
            sb.append(trajectoryEquation);
            
            return sb.toString();
        }
    }
    
    public TurretTrajectoryTester() {
        this(5000.0, 200.0);
    }
    
    public TurretTrajectoryTester(double maxFlywheelRPM, double defaultSpinRate) {
        CalibrationParameters calibration = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel physics = new PhysicsModel(calibration, projectile);
        HubGeometry hub = new HubGeometry();
        TrajectorySimulator trajSim = new TrajectorySimulator(physics, hub);
        this.solver = new InverseSolver(trajSim);
        this.simulator = new SpinShotSimulator(calibration, projectile);
        this.maxFlywheelRPM = maxFlywheelRPM;
        this.defaultSpinRate = defaultSpinRate;
    }
    
    /**
     * Compute optimal solution accounting for turret position and orientation.
     * Now respects max flywheel RPM constraint.
     */
    public SolutionOutput computeOptimalShot(TestConfig config) {
        SolutionOutput output = new SolutionOutput();
        
        // Calculate distance to target for trajectory equation generation
        double dx = config.targetPosition.x - config.robotPosition.x;
        double dy = config.targetPosition.y - config.robotPosition.y;
        double dz = config.targetPosition.z - config.robotPosition.z;
        
        Vector3D spin = new Vector3D(0, config.spinRate, 0); // Backspin
        
        // Calculate max launch speed from max RPM
        double wheelCircumference = Math.PI * wheelDiameterMeters;
        double maxSpeed = (maxFlywheelRPM * wheelCircumference) / 60.0;
        
        // Try progressively higher speeds starting from a reasonable minimum
        // Most FRC shots need at least MIN_SPEED_FLOOR m/s, but can go up to the max
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        double minSpeed = Math.max(MIN_SPEED_FLOOR, distanceToTarget * MIN_SPEED_FACTOR);
        minSpeed = Math.min(minSpeed, maxSpeed); // Can't exceed max
        
        InverseSolver.SolutionResult bestSolution = null;
        double bestSpeed = 0;
        double bestScore = -Double.MAX_VALUE;
        
        // Try different speeds with finer resolution
        double speedStep = (maxSpeed - minSpeed) / SPEED_SEARCH_STEPS;
        
        for (int i = 0; i <= SPEED_SEARCH_STEPS; i++) {
            double testSpeed = minSpeed + i * speedStep;
            if (testSpeed > maxSpeed) continue;
            
            // Solve for optimal trajectory at this speed
            // Pass robot position (absolute), not relative target
            InverseSolver.SolutionResult solution = solver.solve(
                config.robotPosition, testSpeed, spin);
            
            if (solution != null && solution.isHit() && solution.score > bestScore) {
                bestScore = solution.score;
                bestSolution = solution;
                bestSpeed = testSpeed;
            }
        }
        
        InverseSolver.SolutionResult solution = bestSolution;
        
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
        
        // Convert speed to shooter RPM
        output.shooterRPM = (bestSpeed / wheelCircumference) * 60.0;
        
        output.spinRate = config.spinRate;
        output.spinAxis = spin.normalize();
        
        // Calculate success probability from risk score
        // Lower risk = higher probability
        output.riskScore = solution.score;
        output.successProbability = Math.max(0.0, Math.min(1.0, 1.0 - solution.score));
        
        // Calculate confidence based on trajectory stability and convergence
        // High confidence = low variation in nearby solutions
        // Factors: entry angle quality, risk score, distance from limits
        double angleQuality = 1.0 - Math.abs(solution.launchPitchDeg - 45.0) / 45.0; // Prefer 45° launches
        double scoreQuality = 1.0 - Math.min(solution.score, 1.0);
        output.confidence = (angleQuality * 0.3 + scoreQuality * 0.7);
        output.confidence = Math.max(0.0, Math.min(1.0, output.confidence));
        
        // Calculate margin of error based on trajectory arc and physics uncertainties
        // Factors: drag uncertainty, spin variation, entry angle
        double distanceToTargetForError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        double dragErrorFactor = 0.02; // ±2% drag coefficient uncertainty
        double spinErrorFactor = 0.05; // ±5% spin efficiency uncertainty
        output.marginOfError = distanceToTargetForError * (dragErrorFactor + spinErrorFactor * (config.spinRate / 300.0));
        
        // Generate full trajectory equation (non-parabolic with all physics effects)
        output.trajectoryEquation = generateTrajectoryEquation(
            config.robotPosition, new Vector3D(dx, dy, dz), bestSpeed, spin, solution);
        
        // Extract trajectory details
        if (solution.trajectory != null) {
            if (solution.trajectory.entryState != null) {
                double vz = solution.trajectory.entryState.velocity.z;
                double speedAtEntry = solution.trajectory.entryState.velocity.magnitude();
                if (speedAtEntry > 0.1) {
                    output.entryAngle = Math.abs(Math.toDegrees(Math.asin(-vz / speedAtEntry)));
                }
            }
            
            output.flightTime = solution.trajectory.getFlightTime();
            output.apexHeight = solution.trajectory.getMaxHeight();
            
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
     * Generate full trajectory equation with all physics effects (non-simplified).
     * Shows the complete non-parabolic equation including drag, Magnus effect, and gravity.
     */
    private String generateTrajectoryEquation(Vector3D robotPos, Vector3D relativeTarget, 
                                             double speed, Vector3D spin, InverseSolver.SolutionResult solution) {
        StringBuilder eq = new StringBuilder();
        
        // Get physics constants and parameters
        CalibrationParameters params = new CalibrationParameters();
        ProjectileProperties props = new ProjectileProperties();
        double g = PhysicsConstants.GRAVITY;
        double rho = PhysicsConstants.AIR_DENSITY;
        double Cd = params.dragCoefficient;
        double Cm = params.magnusCoefficient;
        double A = props.crossSectionalArea;
        double m = props.massKg;
        
        // Initial conditions
        double yawRad = Math.toRadians(solution.launchYawDeg);
        double pitchRad = Math.toRadians(solution.launchPitchDeg);
        double v0 = speed;
        double wx = spin.x;
        double wy = spin.y;
        double wz = spin.z;
        
        // Initial velocity components
        double v0x = v0 * Math.cos(pitchRad) * Math.cos(yawRad);
        double v0y = v0 * Math.cos(pitchRad) * Math.sin(yawRad);
        double v0z = v0 * Math.sin(pitchRad);
        
        eq.append("      Full Non-Parabolic Trajectory (RK4 Integration Required):\n");
        eq.append("      \n");
        eq.append("      State Equations (Differential Form):\n");
        eq.append("      ───────────────────────────────────────\n");
        eq.append(String.format("      dx/dt = vₓ(t)\n"));
        eq.append(String.format("      dy/dt = vᵧ(t)\n"));
        eq.append(String.format("      dz/dt = v_z(t)\n"));
        eq.append("      \n");
        eq.append("      Acceleration Components:\n");
        eq.append("      ───────────────────────────────────────\n");
        eq.append(String.format("      dvₓ/dt = -½(ρCₐA/m)|v|vₓ + (Cₘ/m)(ωᵧv_z - ω_zvᵧ)|ω||v|\n"));
        eq.append(String.format("      dvᵧ/dt = -½(ρCₐA/m)|v|vᵧ + (Cₘ/m)(ω_zvₓ - ωₓv_z)|ω||v|\n"));
        eq.append(String.format("      dv_z/dt = -g - ½(ρCₐA/m)|v|v_z + (Cₘ/m)(ωₓvᵧ - ωᵧvₓ)|ω||v|\n"));
        eq.append("      \n");
        eq.append("      Spin Decay:\n");
        eq.append("      ───────────────────────────────────────\n");
        eq.append(String.format("      dω/dt = -kω(1 + |v|/%.1f)\n", PhysicsConstants.SPIN_DECAY_VELOCITY_FACTOR));
        eq.append("      \n");
        eq.append("      Parameters:\n");
        eq.append("      ───────────────────────────────────────\n");
        eq.append(String.format("      Initial Position:  (%.3f, %.3f, %.3f) m\n", 
            robotPos.x, robotPos.y, robotPos.z));
        eq.append(String.format("      Initial Velocity:  v₀ = %.3f m/s\n", v0));
        eq.append(String.format("                         vₓ₀ = %.3f m/s\n", v0x));
        eq.append(String.format("                         vᵧ₀ = %.3f m/s\n", v0y));
        eq.append(String.format("                         v_z₀ = %.3f m/s\n", v0z));
        eq.append(String.format("      Initial Spin:      ω = (%.2f, %.2f, %.2f) rad/s\n", wx, wy, wz));
        eq.append(String.format("                         |ω| = %.2f rad/s\n", spin.magnitude()));
        eq.append(String.format("      Gravity:           g = %.2f m/s²\n", g));
        eq.append(String.format("      Air Density:       ρ = %.3f kg/m³\n", rho));
        eq.append(String.format("      Drag Coefficient:  Cₐ = %.3f\n", Cd));
        eq.append(String.format("      Magnus Coefficient: Cₘ = %.6f\n", Cm));
        eq.append(String.format("      Cross Section:     A = %.6f m²\n", A));
        eq.append(String.format("      Ball Mass:         m = %.4f kg\n", m));
        eq.append(String.format("      Spin Decay Rate:   k = %.4f s⁻¹\n", params.spinDecayRate));
        eq.append("      \n");
        eq.append("      Note: This is a coupled nonlinear system requiring numerical\n");
        eq.append("            integration (RK4). Drag term ~ |v|v creates non-parabolic\n");
        eq.append("            trajectory. Magnus effect adds lateral acceleration\n");
        eq.append("            perpendicular to velocity and spin vectors.\n");
        
        return eq.toString();
    }
    
    /**
     * Read test configurations from file.
     * Supports two formats:
     *   1. Simplified format (as in test-targets.txt):
     *      ROBOT_XY: x, y (shooter height fixed at 0.5m)
     *      CURRENT_TURRET_DEGREE: yaw
     *      CURRENT_HOOD_DEGREE: pitch
     *      LAUNCH_PARAMS: speed, spin (optional - spin only used now)
     *      ---
     *   2. Full format:
     *      ROBOT_POSITION: x, y, z
     *      TARGET_POSITION: x, y, z
     *      TURRET_OFFSET: yaw, pitch
     *      LAUNCH_PARAMS: speed, spin (optional - spin only used now)
     *      ---
     */
    public List<TestConfig> readConfigsFromFile(String filename) throws IOException {
        List<TestConfig> configs = new ArrayList<>();
        
        // Default target at field origin (0, 0, 1.83m) - HUB center
        Vector3D defaultTarget = new Vector3D(0.0, 0.0, 1.83);
        double defaultShooterHeight = 0.5; // meters
        
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            int lineNum = 0;
            
            Vector3D robotPos = null;
            Vector3D targetPos = null;
            Double yawOffset = null;
            Double pitchOffset = null;
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
                    if (robotPos != null && yawOffset != null && pitchOffset != null) {
                        
                        // Use default target if not specified
                        if (targetPos == null) {
                            targetPos = defaultTarget;
                        }
                        
                        double finalSpin = (spin != null) ? spin : defaultSpinRate;
                        
                        configs.add(new TestConfig(
                            robotPos, targetPos, yawOffset, pitchOffset, finalSpin));
                    } else {
                        System.err.println("Line " + lineNum + 
                            ": Incomplete scenario (missing required fields)");
                    }
                    
                    // Reset for next scenario
                    robotPos = null;
                    targetPos = null;
                    yawOffset = null;
                    pitchOffset = null;
                    spin = null;
                    continue;
                }
                
                // Parse labeled lines
                try {
                    // Simplified format (ROBOT_XY)
                    if (line.startsWith("ROBOT_XY:")) {
                        String values = line.substring("ROBOT_XY:".length()).trim();
                        String[] parts = values.split(",");
                        if (parts.length == 2) {
                            robotPos = new Vector3D(
                                Double.parseDouble(parts[0].trim()),
                                Double.parseDouble(parts[1].trim()),
                                defaultShooterHeight
                            );
                        }
                    } 
                    // Full format (ROBOT_POSITION)
                    else if (line.startsWith("ROBOT_POSITION:")) {
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
                    } 
                    // Simplified format (CURRENT_TURRET_DEGREE)
                    else if (line.startsWith("CURRENT_TURRET_DEGREE:")) {
                        String value = line.substring("CURRENT_TURRET_DEGREE:".length()).trim();
                        yawOffset = Double.parseDouble(value);
                    }
                    // Simplified format (CURRENT_HOOD_DEGREE)
                    else if (line.startsWith("CURRENT_HOOD_DEGREE:")) {
                        String value = line.substring("CURRENT_HOOD_DEGREE:".length()).trim();
                        pitchOffset = Double.parseDouble(value);
                    }
                    // Full format (TURRET_OFFSET)
                    else if (line.startsWith("TURRET_OFFSET:")) {
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
                            // First parameter (speed) is now ignored, only spin is used
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
        System.out.println("Max flywheel RPM: " + maxFlywheelRPM);
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
        String filename = args.length > 0 ? args[0] : "test-targets.txt";
        double maxRPM = args.length > 1 ? Double.parseDouble(args[1]) : 5000.0;
        double spin = args.length > 2 ? Double.parseDouble(args[2]) : 200.0;
        
        TurretTrajectoryTester tester = new TurretTrajectoryTester(maxRPM, spin);
        tester.processFile(filename);
    }
}
