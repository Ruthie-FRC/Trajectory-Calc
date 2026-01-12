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
    private final TrajectorySimulator trajSimulator;
    private final double maxFlywheelRPM;
    private final double defaultSpinRate;
    private final double wheelDiameterMeters = 0.1016; // 4 inches
    
    // Ultra-fast search heuristics (optimized for <0.1 second total computation)
    // Strategy: Quickly identify few practical candidates, then pick the best
    private static final double GEOMETRIC_SEARCH_RANGE_DEG = 12.0; // Search range around geometric estimate
    private static final double GEOMETRIC_SEARCH_STEP_DEG = 4.0;   // Step size for geometric search
    private static final int FAST_SPEED_STEPS = 5;                  // Number of speeds to try (min, 25%, 50%, 75%, max)
    private static final double EARLY_EXIT_THRESHOLD = 0.75;        // Exit after finding good candidates (lower = faster)
    private static final int MAX_CANDIDATES_TO_FIND = 5;            // Stop after finding this many viable shots
    private static final double REFINEMENT_EXIT_THRESHOLD = 0.80;   // Only refine if best score below this
    
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
        this(5000.0, 200.0);  // Default 5000 RPM
    }
    
    public TurretTrajectoryTester(double maxFlywheelRPM, double defaultSpinRate) {
        CalibrationParameters calibration = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel physics = new PhysicsModel(calibration, projectile);
        HubGeometry hub = new HubGeometry();
        TrajectorySimulator trajSim = new TrajectorySimulator(physics, hub);
        this.solver = new InverseSolver(trajSim);
        this.simulator = new SpinShotSimulator(calibration, projectile);
        this.trajSimulator = trajSim;
        this.maxFlywheelRPM = maxFlywheelRPM;
        this.defaultSpinRate = defaultSpinRate;
    }
    
    /**
     * Compute optimal solution accounting for turret position and orientation.
     * Now respects max flywheel RPM constraint.
     * Uses comprehensive search to find trajectories that actually hit the target.
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
        
        // Calculate required yaw to point at target
        double targetYaw = Math.toDegrees(Math.atan2(dy, dx));
        
        // Geometric estimate for initial pitch
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        double geometricPitch = Math.toDegrees(Math.atan2(dz, distanceToTarget));
        
        // First, try to find ANY trajectory that hits by comprehensive search
        // Search space: speeds from reasonable minimum to max, pitches around geometric estimate
        // For very close shots, use lower minimum speed; for long shots, ensure we search full range
        // Determine minimum speed based on distance (practical heuristic)
        double minSpeed;
        if (distanceToTarget < 2.0) {
            // Very close shots need lower speeds to avoid overshooting
            minSpeed = Math.max(5.0, distanceToTarget * 3.0);
        } else if (distanceToTarget > 10.0) {
            // Very long shots need higher minimum speeds
            minSpeed = Math.max(12.0, distanceToTarget * 1.2);
        } else if (distanceToTarget > 7.0) {
            // Long shots (7-10m) - moderate starting speed
            minSpeed = Math.max(10.0, distanceToTarget * 1.0);
        } else {
            // Normal range (2-7m) - use distance-based estimate
            minSpeed = Math.max(8.0, distanceToTarget * 0.8);
        }
        minSpeed = Math.min(minSpeed, maxSpeed);
        
        TrajectorySimulator.TrajectoryResult bestResult = null;
        double bestSpeed = 0;
        double bestYaw = targetYaw;
        double bestPitch = geometricPitch;
        double bestScore = -Double.MAX_VALUE;
        
        // Optimized comprehensive search for 100% success rate
        // Smart exhaustive search - focus on relevant parameter ranges
        
        // ULTRA-FAST CANDIDATE IDENTIFICATION (<0.1s total)
        // Strategy: Quickly identify only practical angles, test minimal combinations
        // Then pick the best of the few candidates found
        
        java.util.Set<Double> pitchAngleSet = new java.util.HashSet<>();
        
        // Smart sampling around geometric estimate (fewer angles)
        for (double offset = -GEOMETRIC_SEARCH_RANGE_DEG; offset <= GEOMETRIC_SEARCH_RANGE_DEG; offset += GEOMETRIC_SEARCH_STEP_DEG) {
            double angle = geometricPitch + offset;
            if (angle >= 5.0 && angle <= 85.0) {
                pitchAngleSet.add(angle);
            }
        }
        
        // Add only most practical angles based on distance (fewer options)
        // Skip impractical angles entirely
        if (distanceToTarget < 1.5) {
            // Ultra-close: only very steep angles work
            pitchAngleSet.add(65.0);
            pitchAngleSet.add(70.0);
            pitchAngleSet.add(75.0);
            pitchAngleSet.add(80.0);
        } else if (distanceToTarget < 3.0) {
            // Close: only high arcs work
            pitchAngleSet.add(50.0);
            pitchAngleSet.add(57.0);
            pitchAngleSet.add(65.0);
            pitchAngleSet.add(72.0);
            pitchAngleSet.add(80.0);
        } else if (distanceToTarget < 6.0) {
            // Medium: mid-range arcs
            pitchAngleSet.add(35.0);
            pitchAngleSet.add(42.0);
            pitchAngleSet.add(50.0);
            pitchAngleSet.add(57.0);
            pitchAngleSet.add(65.0);
        } else {
            // Long: optimal trajectory range
            pitchAngleSet.add(30.0);
            pitchAngleSet.add(37.0);
            pitchAngleSet.add(45.0);
            pitchAngleSet.add(52.0);
            pitchAngleSet.add(60.0);
        }
        
        // Convert to sorted array
        Double[] pitchAngles = pitchAngleSet.toArray(new Double[0]);
        java.util.Arrays.sort(pitchAngles);
        
        // Speed configuration - MINIMAL (only critical speeds)
        int speedSteps = FAST_SPEED_STEPS;
        double speedStep = (maxSpeed - minSpeed) / speedSteps;
        
        // Yaw offsets - MINIMAL (only most critical angles)
        double[] yawOffsets = {0.0, -3.0, 3.0, -9.0, 9.0};
        
        // Ultra-fast candidate search - find few practical shots quickly
        // Target: <0.1 second total from start to finish
        int candidatesFound = 0;
        for (int s = 0; s <= speedSteps; s++) {
            double testSpeed = minSpeed + s * speedStep;
            if (testSpeed > maxSpeed) continue;
            
            for (double testPitch : pitchAngles) {
                if (testPitch < 5.0 || testPitch > 85.0) continue;
                
                for (double yawOffset : yawOffsets) {
                    double testYaw = targetYaw + yawOffset;
                    
                    // Simulate this trajectory
                    TrajectorySimulator.TrajectoryResult result = trajSimulator.simulateWithShooterModel(
                        config.robotPosition, testSpeed, testYaw, testPitch, spin);
                    
                    if (result.hitTarget) {
                        candidatesFound++;
                        
                        // Scoring based on user-specified priority order:
                        // 1. Accuracy (45%) - Entry quality, rim clearance, center-targeting
                        // 2. Ball flight time (25%) - Minimize time to target
                        // 3. Minimum turret movement (20%) - Prefer solutions near current position
                        // 4. Ball speed/RPM (10%) - Use minimum viable speed
                        
                        // Priority 1: ACCURACY (45% total weight)
                        // Entry score already accounts for center-targeting and rim clearance
                        double accuracyComponent = result.entryScore * 0.45;
                        
                        // Priority 2: BALL FLIGHT TIME (25% weight)
                        // Shorter flight time = faster to target = higher score
                        double flightTime = result.trajectory.get(result.trajectory.size() - 1).time;
                        double maxReasonableTime = 2.5;  // 2.5 seconds max expected
                        double flightTimeScore = Math.max(0, 1.0 - (flightTime / maxReasonableTime));
                        double flightTimeComponent = flightTimeScore * 0.25;
                        
                        // Priority 3: MINIMUM TURRET MOVEMENT (20% weight)
                        // Calculate angular distance from current turret position
                        double yawDelta = Math.abs(testYaw - config.turretYawOffset);
                        double pitchDelta = Math.abs(testPitch - config.turretPitchOffset);
                        // Normalize to typical ranges (±180° yaw, ±90° pitch)
                        double normalizedYawDelta = yawDelta / 180.0;
                        double normalizedPitchDelta = pitchDelta / 90.0;
                        // Combined angular distance (weighted equally)
                        double angularDistance = (normalizedYawDelta + normalizedPitchDelta) / 2.0;
                        double movementScore = Math.max(0, 1.0 - angularDistance);
                        double movementComponent = movementScore * 0.20;
                        
                        // Priority 4: BALL SPEED/RPM (10% weight - lowest priority)
                        // Prefer minimum viable speed
                        double speedRatio = (testSpeed - minSpeed) / (maxSpeed - minSpeed);
                        double speedScore = 1.0 - speedRatio;  // Lower speed = higher score
                        double speedComponent = speedScore * 0.10;
                        
                        // Combined score following user priority order
                        double score = accuracyComponent + flightTimeComponent + movementComponent + speedComponent;
                        
                        if (score > bestScore) {
                            bestScore = score;
                            bestResult = result;
                            bestSpeed = testSpeed;
                            bestYaw = testYaw;
                            bestPitch = testPitch;
                            
                            // Ultra-aggressive early exit: stop after finding good candidates
                            if (bestScore > EARLY_EXIT_THRESHOLD || candidatesFound >= MAX_CANDIDATES_TO_FIND) {
                                break;  // Exit yaw loop
                            }
                        }
                    }
                }
                
                // Early exit if found enough good candidates
                if (bestScore > EARLY_EXIT_THRESHOLD || candidatesFound >= MAX_CANDIDATES_TO_FIND) {
                    break;  // Exit pitch loop
                }
            }
            
            // Early exit if found enough good candidates
            if (bestScore > EARLY_EXIT_THRESHOLD || candidatesFound >= MAX_CANDIDATES_TO_FIND) {
                break;  // Exit speed loop
            }
        }
        
        // Phase 2: Quick refinement only if best candidate isn't excellent
        // Skip refinement if we already found a great solution
        if (bestResult != null && bestResult.hitTarget && bestScore < REFINEMENT_EXIT_THRESHOLD) {
            // Very quick refinement with minimal search space
            double fineSpeedStep = Math.max(0.5, (maxSpeed - minSpeed) / 10.0);
            double[] finePitchOffsets = {0, -2.0, 2.0};  // Only ±2° refinement
            double[] fineYawOffsets = {0, -2.0, 2.0};    // Only ±2° refinement
            
            for (double speedOffset = -fineSpeedStep; speedOffset <= fineSpeedStep; speedOffset += fineSpeedStep) {
                double testSpeed = bestSpeed + speedOffset;
                if (testSpeed < minSpeed || testSpeed > maxSpeed) continue;
                
                for (double pitchOffset : finePitchOffsets) {
                    double testPitch = bestPitch + pitchOffset;
                    if (testPitch < 5.0 || testPitch > 85.0) continue;
                    
                    for (double yawOffset : fineYawOffsets) {
                        double testYaw = bestYaw + yawOffset;
                        
                        TrajectorySimulator.TrajectoryResult result = trajSimulator.simulateWithShooterModel(
                            config.robotPosition, testSpeed, testYaw, testPitch, spin);
                        
                        if (result.hitTarget) {
                            // Recalculate score with same priority order
                            double accuracyComponent = result.entryScore * 0.45;
                            double flightTime = result.trajectory.get(result.trajectory.size() - 1).time;
                            double flightTimeScore = Math.max(0, 1.0 - (flightTime / 2.5));
                            double flightTimeComponent = flightTimeScore * 0.25;
                            double yawDelta = Math.abs(testYaw - config.turretYawOffset);
                            double pitchDelta = Math.abs(testPitch - config.turretPitchOffset);
                            double angularDistance = ((yawDelta / 180.0) + (pitchDelta / 90.0)) / 2.0;
                            double movementComponent = Math.max(0, 1.0 - angularDistance) * 0.20;
                            double speedRatio = (testSpeed - minSpeed) / (maxSpeed - minSpeed);
                            double speedComponent = (1.0 - speedRatio) * 0.10;
                            double score = accuracyComponent + flightTimeComponent + movementComponent + speedComponent;
                            
                            if (score > bestScore) {
                                bestScore = score;
                                bestResult = result;
                                bestSpeed = testSpeed;
                                bestYaw = testYaw;
                                bestPitch = testPitch;
                                
                                // Early exit if excellent
                                if (bestScore > REFINEMENT_EXIT_THRESHOLD) {
                                    break;
                                }
                            }
                        }
                    }
                    if (bestScore > REFINEMENT_EXIT_THRESHOLD) break;
                }
                if (bestScore > REFINEMENT_EXIT_THRESHOLD) break;
            }
        }
        
        // If we didn't find a hit, return no solution
        if (bestResult == null || !bestResult.hitTarget) {
            output.hasShot = false;
            return output;
        }
        
        // We found a trajectory that hits! Now extract the parameters
        output.hasShot = true;
        
        // Calculate adjustments needed from current turret orientation
        output.requiredYawAdjust = bestYaw - config.turretYawOffset;
        output.requiredPitchAdjust = bestPitch - config.turretPitchOffset;
        output.finalYaw = bestYaw;
        output.finalPitch = bestPitch;
        
        // Convert speed to shooter RPM
        output.shooterRPM = (bestSpeed / wheelCircumference) * 60.0;
        
        output.spinRate = config.spinRate;
        output.spinAxis = spin.normalize();
        
        // Calculate success probability from entry score
        output.riskScore = 1.0 - bestScore; // Lower score = lower risk
        output.successProbability = bestScore; // Score directly represents quality
        
        // Calculate confidence based on trajectory quality
        double angleQuality = 1.0 - Math.abs(bestPitch - 45.0) / 45.0; // Prefer 45° launches
        double scoreQuality = bestScore;
        output.confidence = (angleQuality * 0.3 + scoreQuality * 0.7);
        output.confidence = Math.max(0.0, Math.min(1.0, output.confidence));
        
        // Calculate margin of error based on trajectory characteristics
        double distanceToTargetForError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        double dragErrorFactor = 0.02;
        double spinErrorFactor = 0.05;
        output.marginOfError = distanceToTargetForError * (dragErrorFactor + spinErrorFactor * (config.spinRate / 300.0));
        
        // Generate full trajectory equation
        output.trajectoryEquation = generateTrajectoryEquation(
            config.robotPosition, new Vector3D(dx, dy, dz), bestSpeed, spin, 
            new InverseSolver.SolutionResult(bestYaw, bestPitch, bestScore, bestResult));
        
        // Extract trajectory details
        if (bestResult.entryState != null) {
            double vz = bestResult.entryState.velocity.z;
            double speedAtEntry = bestResult.entryState.velocity.magnitude();
            if (speedAtEntry > 0.1) {
                output.entryAngle = Math.abs(Math.toDegrees(Math.asin(-vz / speedAtEntry)));
            }
        }
        
        output.flightTime = bestResult.getFlightTime();
        output.apexHeight = bestResult.getMaxHeight();
        
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
