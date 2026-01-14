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
    
    // Optimized search for 100% success rate with FAST computation (<500ms target)
    // Strategy: Aggressive early exit with hard limits, balanced for close range
    private static final double GEOMETRIC_SEARCH_RANGE_DEG = 10.0; // Narrower search
    private static final double GEOMETRIC_SEARCH_STEP_DEG = 5.0;   // Coarser for speed
    private static final int FAST_SPEED_STEPS = 4;                 // Fewer steps
    private static final double EARLY_EXIT_THRESHOLD = 0.70;       // More aggressive exit
    private static final int MAX_CANDIDATES_TO_FIND = 1;           // Exit after 1 good candidate
    private static final double REFINEMENT_EXIT_THRESHOLD = 0.78;  // Skip refinement earlier
    private static final int MAX_ITERATIONS_BEFORE_EXIT = 120;     // Lower hard limit
    private static final int MAX_CANDIDATES_BEFORE_EXIT = 2;      // Exit after 2 candidates in fast search
    
    // --- SEARCH PARAMETERS ---
    // Coarse grid: broad, but not too dense
    private static final double COARSE_PITCH_STEP = 3.0;
    private static final double COARSE_YAW_STEP = 3.0;
    private static final int COARSE_SPEED_STEPS = 6;

    // Fine grid: dense, but small region
    private static final double FINE_PITCH_STEP = 0.5;
    private static final double FINE_YAW_STEP = 0.5;
    private static final int FINE_SPEED_STEPS = 2;

    private static final int MAX_TOTAL_ITERATIONS = 200; // hard cap
    private static final double EARLY_EXIT_SCORE = 0.93; // only exit if truly excellent
    
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

        // Deterministic geometric seed
        double dx = config.targetPosition.x - config.robotPosition.x;
        double dy = config.targetPosition.y - config.robotPosition.y;
        double dz = config.targetPosition.z - config.robotPosition.z;
        Vector3D spin = new Vector3D(0, config.spinRate, 0); // Backspin
        double maxSpeed = maxBallSpeedMS;
        double targetYaw = Math.toDegrees(Math.atan2(dy, dx));
        double distanceToTarget = Math.sqrt(dx * dx + dy * dy);
        double geometricPitch = Math.toDegrees(Math.atan2(dz, distanceToTarget));

        double minSpeed;
        // Determine minimum speed based on distance (practical heuristic for 17ft max range)
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
        
        // Minimal grid: geometric seed and ±2°/±2° neighbors for yaw/pitch, ±5% speed
        double[] speedOffsets = {0.0, -0.05, 0.05}; // percent of (maxSpeed-minSpeed)
        double[] yawOffsets = {0.0, -2.0, 2.0};
        double[] pitchOffsets = {0.0, -2.0, 2.0};

        double bestSpeed = minSpeed;
        double bestYaw = targetYaw;
        double bestPitch = geometricPitch;
        double bestScore = -Double.MAX_VALUE;
        TrajectorySimulator.TrajectoryResult bestResult = null;

        for (double speedOffset : speedOffsets) {
            double testSpeed = minSpeed + speedOffset * (maxSpeed - minSpeed);
            if (testSpeed < minSpeed || testSpeed > maxSpeed) continue;
            for (double yawOffset : yawOffsets) {
                double testYaw = targetYaw + yawOffset;
                for (double pitchOffset : pitchOffsets) {
                    double testPitch = geometricPitch + pitchOffset;
                    double maxAllowedPitch = (distanceToTarget < 1.0) ? 89.0 : 85.0;
                    if (testPitch < 5.0 || testPitch > maxAllowedPitch) continue;
                    TrajectorySimulator.TrajectoryResult result = trajSimulator.simulateWithShooterModel(
                        config.robotPosition, testSpeed, testYaw, testPitch, spin);
                    if (result.hitTarget) {
                        double accuracyComponent = result.entryScore * 0.45;
                        double flightTime = result.trajectory.get(result.trajectory.size() - 1).time;
                        double flightTimeScore = Math.max(0, 1.0 - (flightTime / 2.5));
                        double flightTimeComponent = flightTimeScore * 0.25;
                        double yawDelta = Math.abs(testYaw - config.turretYawOffset);
                        double pitchDelta = Math.abs(testPitch - config.turretPitchOffset);
                        double movementScore = Math.max(0, 1.0 - ((yawDelta / 180.0 + pitchDelta / 90.0) / 2.0));
                        double movementComponent = movementScore * 0.20;
                        double speedRatio = (testSpeed - minSpeed) / (maxSpeed - minSpeed);
                        double speedComponent = (1.0 - speedRatio) * 0.10;
                        double score = accuracyComponent + flightTimeComponent + movementComponent + speedComponent;
                        if (score > bestScore) {
                            bestScore = score;
                            bestResult = result;
                            bestSpeed = testSpeed;
                            bestYaw = testYaw;
                            bestPitch = testPitch;
                        }
                    }
                }
            }
        }

        if (bestResult == null || !bestResult.hitTarget) {
            output.hasShot = false;
            return output;
        }

        output.hasShot = true;
        output.requiredYawAdjust = bestYaw - config.turretYawOffset;
        output.requiredPitchAdjust = bestPitch - config.turretPitchOffset;
        output.finalYaw = bestYaw;
        output.finalPitch = bestPitch;
        output.shooterSpeed = bestSpeed;
        output.spinRate = config.spinRate;
        output.spinAxis = spin.normalize();
        output.riskScore = 1.0 - bestScore;
        output.successProbability = bestScore;
        double angleQuality = 1.0 - Math.abs(bestPitch - 45.0) / 45.0;
        double scoreQuality = bestScore;
        output.confidence = (angleQuality * 0.3 + scoreQuality * 0.7);
        output.confidence = Math.max(0.0, Math.min(1.0, output.confidence));
        double distanceToTargetForError = Math.sqrt(dx * dx + dy * dy + dz * dz);
        double dragErrorFactor = 0.02;
        double spinErrorFactor = 0.05;
        output.marginOfError = distanceToTargetForError * (dragErrorFactor + spinErrorFactor * (config.spinRate / 300.0));
        output.trajectoryEquation = generateTrajectoryEquation(
            config.robotPosition, new Vector3D(dx, dy, dz), bestSpeed, spin, 
            new InverseSolver.SolutionResult(bestYaw, bestPitch, bestScore, bestResult));
        if (bestResult.entryState != null) {
            double vz = bestResult.entryState.velocity.z;
            double speedAtEntry = bestResult.entryState.velocity.magnitude();
            if (speedAtEntry > 0.1) {
                output.entryAngle = Math.abs(Math.toDegrees(Math.asin(-vz / speedAtEntry)));
            }
        }
        output.flightTime = bestResult.getFlightTime();
        output.apexHeight = bestResult.getMaxHeight();
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
}
