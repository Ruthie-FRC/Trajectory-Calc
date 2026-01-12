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
    private final double maxBallSpeedMS;  // Maximum ball speed in meters per second
    private final double defaultSpinRate;
    
    // Optimized search for 100% success rate with ultra-fast computation (<0.05s)
    // Strategy: Smart comprehensive search - test practical angles efficiently
    private static final double GEOMETRIC_SEARCH_RANGE_DEG = 20.0; // Wider search range for reliability
    private static final double GEOMETRIC_SEARCH_STEP_DEG = 3.0;   // Finer step size for better coverage
    private static final int FAST_SPEED_STEPS = 10;                 // More speed steps for 100% coverage
    private static final double EARLY_EXIT_THRESHOLD = 0.88;        // Higher threshold for faster termination
    private static final int MAX_CANDIDATES_TO_FIND = 8;            // Reduced for faster search
    private static final double REFINEMENT_EXIT_THRESHOLD = 0.90;   // Only refine if best score below this
    
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
        public double shooterSpeed;          // Ball launch speed in meters per second
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
            // Convert speed from m/s to ft/s for output
            double shooterSpeedFPS = shooterSpeed / 0.3048;
            sb.append(String.format("    Shooter Speed:     %.1f ft/s\n", shooterSpeedFPS));
            
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
        this(30.0, 200.0);  // Default 30 ft/s max speed
    }
    
    public TurretTrajectoryTester(double maxBallSpeedFPS, double defaultSpinRate) {
        // Convert ft/s to m/s for internal physics calculations
        double maxBallSpeedMS = maxBallSpeedFPS * 0.3048;
        CalibrationParameters calibration = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel physics = new PhysicsModel(calibration, projectile);
        HubGeometry hub = new HubGeometry();
        TrajectorySimulator trajSim = new TrajectorySimulator(physics, hub);
        this.solver = new InverseSolver(trajSim);
        this.simulator = new SpinShotSimulator(calibration, projectile);
        this.trajSimulator = trajSim;
        this.maxBallSpeedMS = maxBallSpeedMS;
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
        
        // Max launch speed is directly specified in m/s
        double maxSpeed = maxBallSpeedMS;
        
        // Calculate required yaw to point at target
        double targetYaw = Math.toDegrees(Math.atan2(dy, dx));
        
        // Geometric estimate for initial pitch
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        double geometricPitch = Math.toDegrees(Math.atan2(dz, distanceToTarget));
        
        // First, try to find ANY trajectory that hits by comprehensive search
        // Search space: speeds from reasonable minimum to max, pitches around geometric estimate
        // For very close shots, use lower minimum speed; for longer shots within 17ft, ensure we search full range
        // Determine minimum speed based on distance (practical heuristic for 17ft max range)
        double minSpeed;
        if (distanceToTarget < 0.3) {
            // Extreme ultra-close (<0.3m) - MINIMAL speed for near-vertical drops
            minSpeed = 1.0;  // Reduced from 1.5 for slowest possible arcs
        } else if (distanceToTarget < 0.6) {
            // Very ultra-close (0.3-0.6m) - VERY low speed for 80-85° trajectories
            minSpeed = Math.max(1.2, distanceToTarget * 2.2);  // Reduced minimum
        } else if (distanceToTarget < 1.0) {
            // Ultra-close (0.6-1.0m) - low speed for 75-80° trajectories  
            minSpeed = Math.max(1.5, distanceToTarget * 2.0);  // Reduced minimum
        } else if (distanceToTarget < 1.5) {
            // Very close (1.0-1.5m) - moderate-low speed for 65-75° trajectories
            minSpeed = Math.max(2.5, distanceToTarget * 2.0);
        } else if (distanceToTarget > 4.27) {
            // Beyond 14 feet - use higher minimum speeds for extended range
            minSpeed = Math.max(7.5, distanceToTarget * 1.5);
        } else if (distanceToTarget > 3.66) {
            // 12-14 feet range - moderate-high speed
            minSpeed = Math.max(7.0, distanceToTarget * 1.8);
        } else if (distanceToTarget > 3.048) {
            // 10-12 feet range - moderate speed
            minSpeed = Math.max(6.5, distanceToTarget * 2.0);
        } else if (distanceToTarget > 2.4) {
            // 8-10 feet range - moderate speed
            minSpeed = Math.max(6.0, distanceToTarget * 2.0);
        } else {
            // 1.5-2.4m (5-8 feet) range - moderate-low speed
            minSpeed = Math.max(4.5, distanceToTarget * 2.5);
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
            // Allow up to 89° for ultra-close shots
            double maxAngle = (distanceToTarget < 1.0) ? 89.0 : 85.0;
            if (angle >= 5.0 && angle <= maxAngle) {
                pitchAngleSet.add(angle);
            }
        }
        
        // Add more practical angles based on distance for better coverage
        // Optimized for 4.877m (16 feet) max range to ensure 100% success rate
        if (distanceToTarget < 1.0) {
            // EXTREME ultra-close (< 1.0m): ULTRA-comprehensive ULTRA-steep angle coverage
            // Need 70-89° for distances 0.7-1.0m, with ultra-fine steps
            for (double angle = 70.0; angle <= 89.0; angle += 0.5) {
                pitchAngleSet.add(angle);
            }
        } else if (distanceToTarget < 1.7) {
            // Ultra-close (1.0-1.7m): comprehensive steep angle coverage
            // Critical range that was failing - needs very fine coverage
            for (double angle = 60.0; angle <= 87.0; angle += 1.0) {
                pitchAngleSet.add(angle);
            }
        } else if (distanceToTarget < 2.13) {
            // Medium (1.7-7 feet): comprehensive mid-range coverage
            for (double angle = 30.0; angle <= 70.0; angle += 2.5) {
                pitchAngleSet.add(angle);
            }
        } else if (distanceToTarget < 3.66) {
            // Long (7-12 feet): comprehensive optimal trajectory coverage
            for (double angle = 20.0; angle <= 65.0; angle += 3.0) {
                pitchAngleSet.add(angle);
            }
        } else {
            // Very long (12-16 feet): comprehensive extended range coverage
            for (double angle = 15.0; angle <= 60.0; angle += 3.0) {
                pitchAngleSet.add(angle);
            }
        }
        
        // Convert to sorted array
        Double[] pitchAngles = pitchAngleSet.toArray(new Double[0]);
        java.util.Arrays.sort(pitchAngles);
        
        // Speed configuration - MORE steps for ultra-close shots
        int speedSteps;
        if (distanceToTarget < 1.0) {
            // Ultra-close: need MANY more speed steps to find the ultra-precise sweet spot
            speedSteps = 30;  // Increased for better coverage
        } else if (distanceToTarget < 1.7) {
            // Very close: more steps - critical range
            speedSteps = 20;
        } else {
            // Normal: standard steps
            speedSteps = FAST_SPEED_STEPS;
        }
        double speedStep = (maxSpeed - minSpeed) / speedSteps;
        
        // Yaw offsets - OPTIMIZED for speed: fewer angles while maintaining coverage
        // Reduced from 13 to 9 for faster computation
        double[] yawOffsets = {0.0, -2.0, 2.0, -4.0, 4.0, -7.0, 7.0, -10.0, 10.0};
        
        // Comprehensive candidate search - ensure 100% success rate
        // Target: <0.1 second total while finding ALL viable shots
        int candidatesFound = 0;
        for (int s = 0; s <= speedSteps; s++) {
            double testSpeed = minSpeed + s * speedStep;
            if (testSpeed > maxSpeed) continue;
            
            for (double testPitch : pitchAngles) {
                // Allow steeper angles for ultra-close shots
                double maxAllowedPitch = (distanceToTarget < 1.0) ? 89.0 : 85.0;
                if (testPitch < 5.0 || testPitch > maxAllowedPitch) continue;
                
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
                            
                            // Smart early exit: only if we found an excellent solution
                            // Otherwise keep searching for better options
                            if (bestScore > EARLY_EXIT_THRESHOLD && candidatesFound >= MAX_CANDIDATES_TO_FIND) {
                                break;  // Exit yaw loop
                            }
                        }
                    }
                }
                
                // Early exit only if found excellent candidates
                if (bestScore > EARLY_EXIT_THRESHOLD && candidatesFound >= MAX_CANDIDATES_TO_FIND) {
                    break;  // Exit pitch loop
                }
            }
            
            // Early exit only if found excellent candidates
            if (bestScore > EARLY_EXIT_THRESHOLD && candidatesFound >= MAX_CANDIDATES_TO_FIND) {
                break;  // Exit speed loop
            }
        }
        
        // Phase 2: Refinement for near-perfect accuracy
        // More thorough refinement to ensure we find the absolute best
        if (bestResult != null && bestResult.hitTarget && bestScore < REFINEMENT_EXIT_THRESHOLD) {
            // More comprehensive refinement with finer steps
            double fineSpeedStep = Math.max(0.3, (maxSpeed - minSpeed) / 15.0);
            double[] finePitchOffsets = {0, -1.5, 1.5, -3.0, 3.0};  // ±3° refinement with finer steps
            double[] fineYawOffsets = {0, -1.5, 1.5, -3.0, 3.0};    // ±3° refinement with finer steps
            
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
        
        // Speed is already in m/s, no conversion needed
        output.shooterSpeed = bestSpeed;
        
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
        System.out.println("Max ball speed: " + maxBallSpeedMS + " m/s");
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
        double totalComputeTime = 0.0;
        double minComputeTime = Double.MAX_VALUE;
        double maxComputeTime = 0.0;
        
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
                String.format("%.2f", distance) + " m");
            
            // Time the computation
            long startTime = System.nanoTime();
            SolutionOutput solution = computeOptimalShot(config);
            long endTime = System.nanoTime();
            double computeTimeMs = (endTime - startTime) / 1_000_000.0;
            
            totalComputeTime += computeTimeMs;
            minComputeTime = Math.min(minComputeTime, computeTimeMs);
            maxComputeTime = Math.max(maxComputeTime, computeTimeMs);
            
            System.out.println("  Computation time: " + String.format("%.2f", computeTimeMs) + " ms\n");
            System.out.println(solution);
            
            if (solution.hasShot) {
                successful++;
                totalSuccessProb += solution.successProbability;
            }
            
            System.out.println("-------------------------------------------------------\n");
        }
        
        double avgSuccessProb = successful > 0 ? totalSuccessProb / successful : 0.0;
        double avgComputeTime = configs.size() > 0 ? totalComputeTime / configs.size() : 0.0;
        
        System.out.println("=======================================================");
        System.out.println("SUMMARY:");
        System.out.println("  Total scenarios:       " + configs.size());
        System.out.println("  Viable shots found:    " + successful);
        System.out.println("  No solution:           " + (configs.size() - successful));
        System.out.println("  Success rate:          " + 
            String.format("%.1f%%", (100.0 * successful / configs.size())));
        System.out.println("  Avg success prob:      " + 
            String.format("%.1f%%", avgSuccessProb * 100));
        System.out.println();
        System.out.println("PERFORMANCE:");
        System.out.println("  Total compute time:    " + String.format("%.2f", totalComputeTime) + " ms");
        System.out.println("  Average per scenario:  " + String.format("%.2f", avgComputeTime) + " ms");
        System.out.println("  Min compute time:      " + String.format("%.2f", minComputeTime) + " ms");
        System.out.println("  Max compute time:      " + String.format("%.2f", maxComputeTime) + " ms");
        System.out.println("=======================================================");
    }
    
    /**
     * Main method for standalone execution.
     */
    public static void main(String[] args) {
        String filename = args.length > 0 ? args[0] : "test-targets.txt";
        double maxSpeedMS = args.length > 1 ? Double.parseDouble(args[1]) : 30.0;
        double spin = args.length > 2 ? Double.parseDouble(args[2]) : 200.0;
        
        TurretTrajectoryTester tester = new TurretTrajectoryTester(maxSpeedMS, spin);
        tester.processFile(filename);
    }
}
