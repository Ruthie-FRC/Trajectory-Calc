package frc.robot.trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Software-only testing utility for simulating hooded-turret shots.
 * 
 * This class allows comprehensive trajectory analysis by:
 * - Simulating all possible turret orientations within allowed ranges
 * - Computing trajectories using full physics (drag, Magnus, spin, collisions)
 * - Scoring trajectories based on bounce-out risk and rim proximity
 * - Selecting the optimal trajectory with lowest risk
 * - Providing detailed reports including position, velocity, spin over time
 * 
 * Usage:
 * <pre>
 * {@code
 * SpinShotSimulator simulator = new SpinShotSimulator();
 * Vector3D target = new Vector3D(3.0, 0.0, 2.5);
 * SimulationResult result = simulator.simulateShot(target, 12.0, new Vector3D(0, 200, 0));
 * 
 * System.out.println("Best trajectory score: " + result.bestScore);
 * System.out.println("Flight time: " + result.flightTime + "s");
 * System.out.println("Apex height: " + result.apexHeight + "m");
 * }
 * </pre>
 */
public class SpinShotSimulator {
    
    private final CalibrationParameters calibrationParams;
    private final ProjectileProperties projectileProps;
    private final HubGeometry hubGeometry;
    
    // Turret orientation ranges (can be configured)
    private double minYawDeg = -180.0;
    private double maxYawDeg = 180.0;
    private double yawStepDeg = 5.0;
    
    private double minPitchDeg = 10.0;
    private double maxPitchDeg = 70.0;
    private double pitchStepDeg = 2.0;
    
    /**
     * Create simulator with default parameters.
     */
    public SpinShotSimulator() {
        this(new CalibrationParameters(), new ProjectileProperties());
    }
    
    /**
     * Create simulator with custom parameters.
     */
    public SpinShotSimulator(CalibrationParameters calibrationParams, ProjectileProperties projectileProps) {
        this.calibrationParams = calibrationParams;
        this.projectileProps = projectileProps;
        this.hubGeometry = new HubGeometry();
    }
    
    /**
     * Configure turret yaw range.
     */
    public void setYawRange(double minDeg, double maxDeg, double stepDeg) {
        this.minYawDeg = minDeg;
        this.maxYawDeg = maxDeg;
        this.yawStepDeg = stepDeg;
    }
    
    /**
     * Configure turret pitch (elevation) range.
     */
    public void setPitchRange(double minDeg, double maxDeg, double stepDeg) {
        this.minPitchDeg = minDeg;
        this.maxPitchDeg = maxDeg;
        this.pitchStepDeg = stepDeg;
    }
    
    /**
     * Simulate shot to target with full trajectory analysis.
     * 
     * @param target Target position (x, y, z) in meters
     * @param speed Initial launch speed in m/s
     * @param spin Spin vector (rad/s) - magnitude and direction
     * @return Detailed simulation result with optimal trajectory
     */
    public SimulationResult simulateShot(Vector3D target, double speed, Vector3D spin) {
        return simulateShot(target, speed, spin, null);
    }
    
    /**
     * Simulate shot with optional AdvantageKit logging.
     * 
     * @param target Target position (x, y, z) in meters
     * @param speed Initial launch speed in m/s
     * @param spin Spin vector (rad/s) - magnitude and direction
     * @param logPrefix Prefix for AdvantageKit logs (e.g., "Shooter/Simulation/"), or null to disable logging
     * @return Detailed simulation result with optimal trajectory
     */
    public SimulationResult simulateShot(Vector3D target, double speed, Vector3D spin, String logPrefix) {
        long startTime = System.currentTimeMillis();
        
        // Create physics model with configured parameters
        PhysicsModel physicsModel = new PhysicsModel(calibrationParams, projectileProps);
        TrajectorySimulator simulator = new TrajectorySimulator(physicsModel, hubGeometry);
        
        // Search for best trajectory across all turret orientations
        double bestScore = Double.NEGATIVE_INFINITY;
        double bestYaw = 0.0;
        double bestPitch = 0.0;
        TrajectorySimulator.TrajectoryResult bestTrajectory = null;
        
        int simulationCount = 0;
        
        for (double yaw = minYawDeg; yaw <= maxYawDeg; yaw += yawStepDeg) {
            for (double pitch = minPitchDeg; pitch <= maxPitchDeg; pitch += pitchStepDeg) {
                simulationCount++;
                
                // Simulate trajectory with this orientation
                TrajectorySimulator.TrajectoryResult result = simulator.simulateWithShooterModel(
                    target, speed, yaw, pitch, spin
                );
                
                // Score this trajectory
                double score = evaluateTrajectory(result);
                
                // Update best if this is better
                if (score > bestScore) {
                    bestScore = score;
                    bestYaw = yaw;
                    bestPitch = pitch;
                    bestTrajectory = result;
                }
            }
        }
        
        long elapsedTime = System.currentTimeMillis() - startTime;
        
        // Build comprehensive result
        SimulationResult result;
        if (bestTrajectory == null) {
            result = new SimulationResult(false, 0, 0, Double.NEGATIVE_INFINITY, null, 0, 0, 0, null, null, simulationCount);
        } else {
            result = buildSimulationResult(bestYaw, bestPitch, bestScore, bestTrajectory, simulationCount);
        }
        
        // Log to AdvantageKit if requested
        if (logPrefix != null && !logPrefix.isEmpty()) {
            logToAdvantageKit(logPrefix, target, speed, spin, result, elapsedTime);
        }
        
        return result;
    }
    
    /**
     * Log simulation results to AdvantageKit.
     * Uses reflection to avoid hard dependency on AdvantageKit.
     */
    private void logToAdvantageKit(String prefix, Vector3D target, double speed, Vector3D spin,
                                   SimulationResult result, long elapsedMs) {
        try {
            Class<?> loggerClass = Class.forName("org.littletonrobotics.junction.Logger");
            java.lang.reflect.Method recordOutput = loggerClass.getMethod("recordOutput", String.class, double.class);
            java.lang.reflect.Method recordOutputBoolean = loggerClass.getMethod("recordOutput", String.class, boolean.class);
            java.lang.reflect.Method recordOutputString = loggerClass.getMethod("recordOutput", String.class, String.class);
            
            String p = prefix.endsWith("/") ? prefix : prefix + "/";
            
            // Log inputs
            recordOutput.invoke(null, p + "Input/TargetX", target.x);
            recordOutput.invoke(null, p + "Input/TargetY", target.y);
            recordOutput.invoke(null, p + "Input/TargetZ", target.z);
            recordOutput.invoke(null, p + "Input/Speed", speed);
            recordOutput.invoke(null, p + "Input/SpinRate", spin.magnitude());
            recordOutput.invoke(null, p + "Input/SpinAxisX", spin.x);
            recordOutput.invoke(null, p + "Input/SpinAxisY", spin.y);
            recordOutput.invoke(null, p + "Input/SpinAxisZ", spin.z);
            
            // Log outputs
            recordOutput.invoke(null, p + "Output/LaunchYaw", result.launchYawDeg);
            recordOutput.invoke(null, p + "Output/LaunchPitch", result.launchPitchDeg);
            recordOutput.invoke(null, p + "Output/RiskScore", result.bestScore);
            recordOutputBoolean.invoke(null, p + "Output/Hit", result.hit);
            recordOutputString.invoke(null, p + "Output/Outcome", result.predictedOutcome);
            
            // Log trajectory metrics
            recordOutput.invoke(null, p + "Metrics/ApexHeight", result.apexHeight);
            recordOutput.invoke(null, p + "Metrics/FlightTime", result.flightTime);
            recordOutput.invoke(null, p + "Metrics/EntryAngle", result.entryAngleDeg);
            recordOutput.invoke(null, p + "Metrics/SimulationsRun", (double) result.simulationsRun);
            recordOutput.invoke(null, p + "Metrics/CalculationTimeMs", (double) elapsedMs);
            
            // Log spin decay
            if (result.spinDecay != null) {
                recordOutput.invoke(null, p + "Spin/InitialRate", result.spinDecay.getInitialSpinRate());
                recordOutput.invoke(null, p + "Spin/FinalRate", result.spinDecay.getFinalSpinRate());
                recordOutput.invoke(null, p + "Spin/DecayPercent", result.spinDecay.getDecayPercentage());
            }
            
        } catch (Exception e) {
            // AdvantageKit not available or error logging - silently ignore
        }
    }
    
    /**
     * Simulate shot with multiple trials including small perturbations to test consistency.
     * 
     * @param target Target position (x, y, z)
     * @param speed Nominal launch speed
     * @param spin Nominal spin vector
     * @param numTrials Number of perturbed trials to run
     * @param speedPerturbation Max speed perturbation (+/- m/s)
     * @param spinPerturbation Max spin perturbation (+/- rad/s)
     * @return Array of simulation results for each trial
     */
    public SimulationResult[] simulateShotWithPerturbations(Vector3D target, double speed, Vector3D spin,
                                                            int numTrials, double speedPerturbation, 
                                                            double spinPerturbation) {
        SimulationResult[] results = new SimulationResult[numTrials];
        
        for (int i = 0; i < numTrials; i++) {
            // Add small random perturbations
            double perturbedSpeed = speed;
            Vector3D perturbedSpin = spin;
            
            if (i > 0) { // First trial is nominal (no perturbation)
                // Deterministic perturbations based on trial index
                double speedDelta = speedPerturbation * Math.sin(i * 0.5);
                double spinDelta = spinPerturbation * Math.cos(i * 0.7);
                
                perturbedSpeed = speed + speedDelta;
                perturbedSpin = new Vector3D(
                    spin.x + spinDelta * 0.1,
                    spin.y + spinDelta,
                    spin.z + spinDelta * 0.1
                );
            }
            
            results[i] = simulateShot(target, perturbedSpeed, perturbedSpin);
        }
        
        return results;
    }
    
    /**
     * Evaluate trajectory quality using same multi-factor scoring as InverseSolver.
     * Higher score is better.
     */
    private double evaluateTrajectory(TrajectorySimulator.TrajectoryResult result) {
        if (!result.hitTarget) {
            // Miss penalty
            if (result.entryState != null) {
                Vector3D center = hubGeometry.getCenter();
                double dx = result.entryState.position.x - center.x;
                double dy = result.entryState.position.y - center.y;
                double missDistance = Math.sqrt(dx * dx + dy * dy);
                return -missDistance;
            }
            return -100.0;
        }
        
        // Base score from entry quality
        double score = result.entryScore;
        
        // Bounce-out risk penalty
        double bounceRisk = calculateBounceOutRisk(result);
        if (bounceRisk > PhysicsConstants.BOUNCE_OUT_RISK_THRESHOLD) {
            score -= bounceRisk * 2.0;
        }
        
        // Rim proximity penalty
        double rimDistance = calculateRimProximity(result);
        if (rimDistance < PhysicsConstants.RIM_DANGER_ZONE) {
            score -= (PhysicsConstants.RIM_DANGER_ZONE - rimDistance) * 5.0;
        }
        
        // Lateral velocity penalty
        if (result.entryState != null) {
            double lateralSpeed = Math.sqrt(
                result.entryState.velocity.x * result.entryState.velocity.x +
                result.entryState.velocity.y * result.entryState.velocity.y
            );
            if (lateralSpeed > PhysicsConstants.DEFAULT_MAX_LATERAL_VELOCITY) {
                score -= (lateralSpeed - PhysicsConstants.DEFAULT_MAX_LATERAL_VELOCITY) * 0.5;
            }
        }
        
        return score;
    }
    
    /**
     * Calculate bounce-out risk (same algorithm as InverseSolver).
     */
    private double calculateBounceOutRisk(TrajectorySimulator.TrajectoryResult result) {
        if (result.entryState == null) {
            return 1.0;
        }
        
        ProjectileState entry = result.entryState;
        
        // Vertical velocity risk
        double vzRisk = 0.0;
        if (entry.velocity.z > -1.0) {
            vzRisk = (1.0 + entry.velocity.z) / 2.0;
        }
        
        // Entry angle risk
        double speed = entry.velocity.magnitude();
        double entryAngle = 0.0;
        if (speed > 0.1) {
            entryAngle = Math.abs(Math.toDegrees(Math.asin(-entry.velocity.z / speed)));
        }
        double angleRisk = (90.0 - entryAngle) / 90.0;
        
        // Spin alignment risk
        double spinRisk = 0.0;
        double spinMagnitude = entry.spin.magnitude();
        if (spinMagnitude > 10.0 && speed > 0.1) {
            double spinAlignment = entry.spin.y / spinMagnitude;
            
            if (spinAlignment > PhysicsConstants.SPIN_ALIGNMENT_THRESHOLD) {
                spinRisk = -PhysicsConstants.SPIN_ALIGNMENT_BONUS * (spinAlignment - PhysicsConstants.SPIN_ALIGNMENT_THRESHOLD);
            } else if (spinAlignment < -PhysicsConstants.SPIN_ALIGNMENT_THRESHOLD) {
                spinRisk = PhysicsConstants.SPIN_ALIGNMENT_PENALTY * Math.abs(spinAlignment);
            }
        }
        
        double baseRisk = vzRisk * 0.6 + angleRisk * 0.4;
        return Math.max(0.0, Math.min(1.0, baseRisk + spinRisk));
    }
    
    /**
     * Calculate minimum distance to rim edge.
     */
    private double calculateRimProximity(TrajectorySimulator.TrajectoryResult result) {
        if (result.entryState == null) {
            return 0.0;
        }
        
        Vector3D entry = result.entryState.position;
        Vector3D center = hubGeometry.getCenter();
        
        double dx = entry.x - center.x;
        double dy = entry.y - center.y;
        double distFromCenter = Math.sqrt(dx * dx + dy * dy);
        
        double rimRadius = hubGeometry.getOpeningRadius();
        double distToRim = rimRadius - distFromCenter;
        
        return Math.max(0, distToRim);
    }
    
    /**
     * Build comprehensive simulation result from best trajectory.
     */
    private SimulationResult buildSimulationResult(double yaw, double pitch, double score,
                                                   TrajectorySimulator.TrajectoryResult trajectory,
                                                   int simulationCount) {
        // Extract trajectory data
        List<ProjectileState> states = trajectory.trajectory;
        
        // Calculate apex height
        double apexHeight = trajectory.getMaxHeight();
        
        // Calculate flight time
        double flightTime = trajectory.getFlightTime();
        
        // Calculate entry angle
        double entryAngleDeg = 0.0;
        if (trajectory.entryState != null) {
            double speed = trajectory.entryState.velocity.magnitude();
            if (speed > 0.1) {
                entryAngleDeg = Math.toDegrees(Math.asin(-trajectory.entryState.velocity.z / speed));
            }
        }
        
        // Extract spin decay curve
        SpinDecayCurve spinCurve = extractSpinDecayCurve(states);
        
        // Build trajectory over time data
        TrajectoryOverTime trajectoryData = extractTrajectoryData(states);
        
        // Determine predicted outcome
        boolean predictedHit = trajectory.hitTarget;
        double bounceRisk = calculateBounceOutRisk(trajectory);
        String outcome = predictedHit ? (bounceRisk < 0.3 ? "HIT" : "HIT_WITH_BOUNCE_RISK") : "MISS";
        
        return new SimulationResult(
            predictedHit,
            yaw,
            pitch,
            score,
            outcome,
            apexHeight,
            flightTime,
            entryAngleDeg,
            spinCurve,
            trajectoryData,
            simulationCount
        );
    }
    
    /**
     * Extract spin decay data from trajectory.
     */
    private SpinDecayCurve extractSpinDecayCurve(List<ProjectileState> states) {
        double[] times = new double[states.size()];
        double[] spinRates = new double[states.size()];
        
        for (int i = 0; i < states.size(); i++) {
            ProjectileState state = states.get(i);
            times[i] = state.time;
            spinRates[i] = state.spin.magnitude();
        }
        
        return new SpinDecayCurve(times, spinRates);
    }
    
    /**
     * Extract full trajectory data (position, velocity, spin over time).
     */
    private TrajectoryOverTime extractTrajectoryData(List<ProjectileState> states) {
        int n = states.size();
        double[] times = new double[n];
        Vector3D[] positions = new Vector3D[n];
        Vector3D[] velocities = new Vector3D[n];
        Vector3D[] spins = new Vector3D[n];
        
        for (int i = 0; i < n; i++) {
            ProjectileState state = states.get(i);
            times[i] = state.time;
            positions[i] = state.position;
            velocities[i] = state.velocity;
            spins[i] = state.spin;
        }
        
        return new TrajectoryOverTime(times, positions, velocities, spins);
    }
    
    /**
     * Result of simulation including full trajectory analysis.
     */
    public static class SimulationResult {
        public final boolean hit;
        public final double launchYawDeg;
        public final double launchPitchDeg;
        public final double bestScore;
        public final String predictedOutcome; // "HIT", "HIT_WITH_BOUNCE_RISK", "MISS"
        
        public final double apexHeight;      // meters
        public final double flightTime;      // seconds
        public final double entryAngleDeg;   // degrees
        
        public final SpinDecayCurve spinDecay;
        public final TrajectoryOverTime trajectory;
        
        public final int simulationsRun;
        
        public SimulationResult(boolean hit, double launchYawDeg, double launchPitchDeg,
                               double bestScore, String predictedOutcome,
                               double apexHeight, double flightTime, double entryAngleDeg,
                               SpinDecayCurve spinDecay, TrajectoryOverTime trajectory,
                               int simulationsRun) {
            this.hit = hit;
            this.launchYawDeg = launchYawDeg;
            this.launchPitchDeg = launchPitchDeg;
            this.bestScore = bestScore;
            this.predictedOutcome = predictedOutcome;
            this.apexHeight = apexHeight;
            this.flightTime = flightTime;
            this.entryAngleDeg = entryAngleDeg;
            this.spinDecay = spinDecay;
            this.trajectory = trajectory;
            this.simulationsRun = simulationsRun;
        }
        
        /**
         * Generate detailed report as formatted string.
         */
        public String generateReport() {
            StringBuilder sb = new StringBuilder();
            sb.append("=== Spin Shot Simulation Report ===\n");
            sb.append(String.format("Simulations Run: %d\n", simulationsRun));
            sb.append(String.format("Best Launch Angles: Yaw=%.2f°, Pitch=%.2f°\n", launchYawDeg, launchPitchDeg));
            sb.append(String.format("Risk Score: %.3f\n", bestScore));
            sb.append(String.format("Predicted Outcome: %s\n", predictedOutcome));
            sb.append(String.format("Apex Height: %.2f m\n", apexHeight));
            sb.append(String.format("Flight Time: %.3f s\n", flightTime));
            sb.append(String.format("Entry Angle: %.1f°\n", entryAngleDeg));
            
            if (spinDecay != null) {
                sb.append(String.format("Initial Spin: %.1f rad/s\n", spinDecay.getInitialSpinRate()));
                sb.append(String.format("Final Spin: %.1f rad/s\n", spinDecay.getFinalSpinRate()));
                sb.append(String.format("Spin Decay: %.1f%%\n", spinDecay.getDecayPercentage()));
            }
            
            if (trajectory != null) {
                sb.append(String.format("Trajectory Points: %d\n", trajectory.times.length));
            }
            
            sb.append("===================================\n");
            return sb.toString();
        }
        
        /**
         * Export to JSON-like string format.
         */
        public String toJSON() {
            StringBuilder json = new StringBuilder();
            json.append("{\n");
            json.append(String.format("  \"hit\": %s,\n", hit));
            json.append(String.format("  \"launchYawDeg\": %.2f,\n", launchYawDeg));
            json.append(String.format("  \"launchPitchDeg\": %.2f,\n", launchPitchDeg));
            json.append(String.format("  \"bestScore\": %.3f,\n", bestScore));
            json.append(String.format("  \"predictedOutcome\": \"%s\",\n", predictedOutcome));
            json.append(String.format("  \"apexHeight\": %.3f,\n", apexHeight));
            json.append(String.format("  \"flightTime\": %.3f,\n", flightTime));
            json.append(String.format("  \"entryAngleDeg\": %.2f,\n", entryAngleDeg));
            json.append(String.format("  \"simulationsRun\": %d\n", simulationsRun));
            json.append("}");
            return json.toString();
        }
    }
    
    /**
     * Spin decay data over time.
     */
    public static class SpinDecayCurve {
        public final double[] times;      // seconds
        public final double[] spinRates;  // rad/s
        
        public SpinDecayCurve(double[] times, double[] spinRates) {
            this.times = times;
            this.spinRates = spinRates;
        }
        
        public double getInitialSpinRate() {
            return spinRates.length > 0 ? spinRates[0] : 0.0;
        }
        
        public double getFinalSpinRate() {
            return spinRates.length > 0 ? spinRates[spinRates.length - 1] : 0.0;
        }
        
        public double getDecayPercentage() {
            double initial = getInitialSpinRate();
            if (initial < 1e-6) return 0.0;
            return 100.0 * (initial - getFinalSpinRate()) / initial;
        }
    }
    
    /**
     * Full trajectory data over time.
     */
    public static class TrajectoryOverTime {
        public final double[] times;       // seconds
        public final Vector3D[] positions; // meters
        public final Vector3D[] velocities; // m/s
        public final Vector3D[] spins;     // rad/s
        
        public TrajectoryOverTime(double[] times, Vector3D[] positions, 
                                 Vector3D[] velocities, Vector3D[] spins) {
            this.times = times;
            this.positions = positions;
            this.velocities = velocities;
            this.spins = spins;
        }
    }
}
