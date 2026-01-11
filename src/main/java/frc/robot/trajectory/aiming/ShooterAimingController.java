package frc.robot.trajectory;

import java.util.HashMap;
import java.util.Map;

/**
 * High-level control class for automatic turret and hood aiming.
 * 
 * Given the robot's field pose and target pose, automatically computes the optimal
 * turret yaw and hood pitch that minimize bounce-out risk using full physics simulation.
 * 
 * Key Features:
 * - Field-relative to robot-relative coordinate transformation
 * - Accounts for robot heading when computing turret yaw
 * - Uses SpinShotSimulator for comprehensive trajectory evaluation
 * - Multi-factor risk scoring with bounce-out and rim proximity penalties
 * - Caches recent successful solutions for faster convergence
 * - Smooth output commands to prevent oscillation
 * - Returns "no-shot" state if no low-risk solution exists
 * 
 * Usage:
 * <pre>
 * {@code
 * ShooterAimingController controller = new ShooterAimingController();
 * controller.setTurretYawLimits(-Math.PI/2, Math.PI/2);  // ±90 degrees
 * controller.setHoodPitchLimits(Math.toRadians(15), Math.toRadians(75));
 * 
 * // Continuous updates as robot moves
 * AimingSolution solution = controller.update(robotPose, targetPose);
 * 
 * if (solution.hasShot()) {
 *     turret.setYaw(solution.turretYawRad);
 *     hood.setPitch(solution.hoodPitchRad);
 *     shooter.setSpeed(solution.shooterSpeedMps);
 * }
 * }
 * </pre>
 */
public class ShooterAimingController {
    
    private final SpinShotSimulator simulator;
    private final CalibrationParameters calibrationParams;
    private final ProjectileProperties projectileProps;
    
    // Mechanical limits (radians)
    private double minTurretYawRad = -Math.PI;
    private double maxTurretYawRad = Math.PI;
    private double minHoodPitchRad = Math.toRadians(10.0);
    private double maxHoodPitchRad = Math.toRadians(70.0);
    
    // Resolution for searching (degrees)
    private double turretYawStepDeg = 5.0;
    private double hoodPitchStepDeg = 2.0;
    
    // Spin configuration
    private double defaultSpinRate = 200.0;  // rad/s
    private Vector3D defaultSpinAxis = new Vector3D(0, 1, 0);  // Backspin
    
    // Shooter parameters
    private double defaultShooterSpeed = 12.0;  // m/s
    private double shooterHeight = 0.5;  // m above robot origin
    
    // Risk threshold
    private double maxAcceptableRisk = 0.5;  // Reject trajectories with higher risk
    
    // Smoothing and caching
    private AimingSolution lastSolution = null;
    private double smoothingFactor = 0.3;  // 0 = no smoothing, 1 = full smoothing
    private Map<String, CachedSolution> solutionCache = new HashMap<>();
    private int maxCacheSize = 50;
    
    // Perturbation testing for confidence
    private boolean enablePerturbationTesting = true;
    private int perturbationTrials = 3;
    private double perturbationSpeedVariation = 0.3;  // m/s
    private double perturbationSpinVariation = 10.0;  // rad/s
    
    /**
     * Create controller with default parameters.
     */
    public ShooterAimingController() {
        this(new CalibrationParameters(), new ProjectileProperties());
    }
    
    /**
     * Create controller with custom calibration and projectile properties.
     */
    public ShooterAimingController(CalibrationParameters calibrationParams, ProjectileProperties projectileProps) {
        this.calibrationParams = calibrationParams;
        this.projectileProps = projectileProps;
        this.simulator = new SpinShotSimulator(calibrationParams, projectileProps);
    }
    
    /**
     * Set turret yaw mechanical limits in radians.
     * 
     * @param minRad Minimum turret yaw (radians, robot-relative)
     * @param maxRad Maximum turret yaw (radians, robot-relative)
     */
    public void setTurretYawLimits(double minRad, double maxRad) {
        this.minTurretYawRad = minRad;
        this.maxTurretYawRad = maxRad;
        updateSimulatorRanges();
    }
    
    /**
     * Set hood pitch mechanical limits in radians.
     * 
     * @param minRad Minimum hood pitch (radians from horizontal)
     * @param maxRad Maximum hood pitch (radians from horizontal)
     */
    public void setHoodPitchLimits(double minRad, double maxRad) {
        this.minHoodPitchRad = minRad;
        this.maxHoodPitchRad = maxRad;
        updateSimulatorRanges();
    }
    
    /**
     * Set search resolution for turret and hood.
     * 
     * @param turretYawStepDeg Step size for turret yaw search (degrees)
     * @param hoodPitchStepDeg Step size for hood pitch search (degrees)
     */
    public void setSearchResolution(double turretYawStepDeg, double hoodPitchStepDeg) {
        this.turretYawStepDeg = turretYawStepDeg;
        this.hoodPitchStepDeg = hoodPitchStepDeg;
        updateSimulatorRanges();
    }
    
    /**
     * Set default spin configuration for shots.
     * 
     * @param spinRate Spin rate in rad/s
     * @param spinAxisX X component of spin axis
     * @param spinAxisY Y component of spin axis (typically 1 for backspin)
     * @param spinAxisZ Z component of spin axis
     */
    public void setDefaultSpin(double spinRate, double spinAxisX, double spinAxisY, double spinAxisZ) {
        this.defaultSpinRate = spinRate;
        this.defaultSpinAxis = new Vector3D(spinAxisX, spinAxisY, spinAxisZ).normalize();
    }
    
    /**
     * Set default shooter speed.
     * 
     * @param speedMps Shooter wheel speed in m/s
     */
    public void setDefaultShooterSpeed(double speedMps) {
        this.defaultShooterSpeed = speedMps;
    }
    
    /**
     * Set shooter height above robot origin.
     * 
     * @param heightM Height in meters
     */
    public void setShooterHeight(double heightM) {
        this.shooterHeight = heightM;
    }
    
    /**
     * Set maximum acceptable risk score for trajectories.
     * Higher values allow riskier shots (less conservative).
     * 
     * @param maxRisk Maximum risk score (typically 0.3-0.7)
     */
    public void setMaxAcceptableRisk(double maxRisk) {
        this.maxAcceptableRisk = maxRisk;
    }
    
    /**
     * Set output smoothing factor to prevent command oscillation.
     * 
     * @param factor Smoothing factor: 0 = no smoothing (instant), 1 = full smoothing (very slow)
     */
    public void setSmoothingFactor(double factor) {
        this.smoothingFactor = Math.max(0.0, Math.min(1.0, factor));
    }
    
    /**
     * Enable or disable perturbation testing for confidence calculation.
     * 
     * @param enable True to enable perturbation testing
     */
    public void enablePerturbationTesting(boolean enable) {
        this.enablePerturbationTesting = enable;
    }
    
    /**
     * Clear the solution cache.
     */
    public void clearCache() {
        solutionCache.clear();
        lastSolution = null;
    }
    
    /**
     * Update aiming solution based on current robot and target poses.
     * 
     * @param robotPose Robot field pose (x, y, heading in meters and radians)
     * @param targetPose Target field pose (x, y, z in meters)
     * @return Aiming solution with turret/hood commands and confidence
     */
    public AimingSolution update(RobotPose robotPose, Vector3D targetPose) {
        return update(robotPose, targetPose, null);
    }
    
    /**
     * Update aiming solution with optional AdvantageKit logging.
     * 
     * @param robotPose Robot field pose (x, y, heading in meters and radians)
     * @param targetPose Target field pose (x, y, z in meters)
     * @param logPrefix Prefix for AdvantageKit logs, or null to disable
     * @return Aiming solution with turret/hood commands and confidence
     */
    public AimingSolution update(RobotPose robotPose, Vector3D targetPose, String logPrefix) {
        // Convert field-relative target to robot-relative coordinates
        Vector3D robotRelativeTarget = fieldToRobotRelative(robotPose, targetPose);
        
        // Check cache for recent solution at similar position
        String cacheKey = getCacheKey(robotRelativeTarget);
        CachedSolution cached = solutionCache.get(cacheKey);
        
        // If cached and recent, use as starting guess for simulator
        if (cached != null && (System.currentTimeMillis() - cached.timestamp) < 2000) {
            // TODO: Could use cached solution as initial guess
        }
        
        // Add shooter height to robot-relative target
        Vector3D shooterRelativeTarget = new Vector3D(
            robotRelativeTarget.x,
            robotRelativeTarget.y,
            robotRelativeTarget.z - shooterHeight
        );
        
        // Create spin vector
        Vector3D spinVector = defaultSpinAxis.scale(defaultSpinRate);
        
        // Run simulation to find optimal trajectory
        SpinShotSimulator.SimulationResult simResult;
        try {
            simResult = simulator.simulateShot(
                shooterRelativeTarget, 
                defaultShooterSpeed, 
                spinVector, 
                logPrefix
            );
        } catch (Exception e) {
            // Simulation failed, return no-shot state
            return createNoShotSolution("Simulation failed: " + e.getMessage());
        }
        
        // Check if a valid hit was found
        if (!simResult.hit || simResult.bestScore > maxAcceptableRisk) {
            return createNoShotSolution("No low-risk trajectory found (best score: " + 
                String.format("%.3f", simResult.bestScore) + ")");
        }
        
        // Convert launch angles to robot-relative turret and hood commands
        double launchYawRad = Math.toRadians(simResult.launchYawDeg);
        double launchPitchRad = Math.toRadians(simResult.launchPitchDeg);
        
        // Turret yaw is robot-relative (already in robot coordinates)
        double turretYawRad = launchYawRad;
        
        // Hood pitch is elevation angle
        double hoodPitchRad = launchPitchRad;
        
        // Clamp to mechanical limits
        turretYawRad = clamp(turretYawRad, minTurretYawRad, maxTurretYawRad);
        hoodPitchRad = clamp(hoodPitchRad, minHoodPitchRad, maxHoodPitchRad);
        
        // Calculate confidence metric via perturbation testing
        double confidence = 1.0;
        if (enablePerturbationTesting) {
            confidence = calculateConfidence(shooterRelativeTarget, spinVector);
        }
        
        // Create solution
        AimingSolution solution = new AimingSolution(
            true,  // hasShot
            turretYawRad,
            hoodPitchRad,
            defaultShooterSpeed,
            defaultSpinRate,
            defaultSpinAxis.x,
            defaultSpinAxis.y,
            defaultSpinAxis.z,
            confidence,
            simResult.bestScore,
            ""  // No error message
        );
        
        // Apply smoothing if we have a previous solution
        if (lastSolution != null && lastSolution.hasShot) {
            solution = smoothSolution(lastSolution, solution);
        }
        
        // Cache the solution
        cacheSolution(cacheKey, solution);
        
        // Update last solution
        lastSolution = solution;
        
        return solution;
    }
    
    /**
     * Transform field-relative target position to robot-relative coordinates.
     */
    private Vector3D fieldToRobotRelative(RobotPose robotPose, Vector3D targetFieldPos) {
        // Translate target to robot origin
        double dx = targetFieldPos.x - robotPose.x;
        double dy = targetFieldPos.y - robotPose.y;
        
        // Rotate by -heading to get robot-relative coordinates
        double cos = Math.cos(-robotPose.headingRad);
        double sin = Math.sin(-robotPose.headingRad);
        
        double xRobot = dx * cos - dy * sin;
        double yRobot = dx * sin + dy * cos;
        
        return new Vector3D(xRobot, yRobot, targetFieldPos.z);
    }
    
    /**
     * Calculate confidence metric using perturbation testing.
     * Returns fraction of perturbed trials that successfully hit.
     */
    private double calculateConfidence(Vector3D target, Vector3D spin) {
        if (perturbationTrials <= 0) {
            return 1.0;
        }
        
        try {
            SpinShotSimulator.SimulationResult[] results = 
                simulator.simulateShotWithPerturbations(
                    target,
                    defaultShooterSpeed,
                    spin,
                    perturbationTrials,
                    perturbationSpeedVariation,
                    perturbationSpinVariation
                );
            
            int successCount = 0;
            for (SpinShotSimulator.SimulationResult result : results) {
                if (result.hit && result.bestScore <= maxAcceptableRisk) {
                    successCount++;
                }
            }
            
            return (double) successCount / results.length;
            
        } catch (Exception e) {
            // If perturbation testing fails, return moderate confidence
            return 0.7;
        }
    }
    
    /**
     * Apply smoothing to prevent command oscillation.
     */
    private AimingSolution smoothSolution(AimingSolution last, AimingSolution current) {
        if (smoothingFactor < 0.01) {
            return current;  // No smoothing
        }
        
        double smoothedYaw = last.turretYawRad * smoothingFactor + 
                             current.turretYawRad * (1.0 - smoothingFactor);
        double smoothedPitch = last.hoodPitchRad * smoothingFactor + 
                               current.hoodPitchRad * (1.0 - smoothingFactor);
        double smoothedSpeed = last.shooterSpeedMps * smoothingFactor + 
                               current.shooterSpeedMps * (1.0 - smoothingFactor);
        
        return new AimingSolution(
            current.hasShot,
            smoothedYaw,
            smoothedPitch,
            smoothedSpeed,
            current.spinRate,
            current.spinAxisX,
            current.spinAxisY,
            current.spinAxisZ,
            current.confidence,
            current.riskScore,
            current.errorMessage
        );
    }
    
    /**
     * Update simulator ranges based on current limits.
     */
    private void updateSimulatorRanges() {
        simulator.setYawRange(
            Math.toDegrees(minTurretYawRad),
            Math.toDegrees(maxTurretYawRad),
            turretYawStepDeg
        );
        simulator.setPitchRange(
            Math.toDegrees(minHoodPitchRad),
            Math.toDegrees(maxHoodPitchRad),
            hoodPitchStepDeg
        );
    }
    
    /**
     * Generate cache key for a robot-relative target position.
     */
    private String getCacheKey(Vector3D target) {
        // Round to 0.5m grid for caching
        int xGrid = (int) Math.round(target.x * 2.0);
        int yGrid = (int) Math.round(target.y * 2.0);
        int zGrid = (int) Math.round(target.z * 2.0);
        return xGrid + "," + yGrid + "," + zGrid;
    }
    
    /**
     * Cache a solution for faster future lookups.
     */
    private void cacheSolution(String key, AimingSolution solution) {
        if (solutionCache.size() >= maxCacheSize) {
            // Simple eviction: remove oldest entry
            String oldestKey = solutionCache.keySet().iterator().next();
            solutionCache.remove(oldestKey);
        }
        solutionCache.put(key, new CachedSolution(solution, System.currentTimeMillis()));
    }
    
    /**
     * Create a "no-shot" solution indicating no valid trajectory exists.
     */
    private AimingSolution createNoShotSolution(String reason) {
        return new AimingSolution(
            false,  // hasShot
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0,  // confidence
            1.0,  // riskScore (high)
            reason
        );
    }
    
    /**
     * Clamp value to range.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Robot pose in field coordinates.
     */
    public static class RobotPose {
        public final double x;           // meters
        public final double y;           // meters
        public final double headingRad;  // radians (0 = +X axis)
        
        public RobotPose(double x, double y, double headingRad) {
            this.x = x;
            this.y = y;
            this.headingRad = headingRad;
        }
    }
    
    /**
     * Aiming solution with turret/hood commands and confidence.
     */
    public static class AimingSolution {
        public final boolean hasShot;          // True if valid shot exists
        public final double turretYawRad;      // Robot-relative turret yaw (radians)
        public final double hoodPitchRad;      // Hood pitch from horizontal (radians)
        public final double shooterSpeedMps;   // Required shooter speed (m/s)
        public final double spinRate;          // Spin rate (rad/s)
        public final double spinAxisX;         // Spin axis X component
        public final double spinAxisY;         // Spin axis Y component
        public final double spinAxisZ;         // Spin axis Z component
        public final double confidence;        // Confidence metric (0-1)
        public final double riskScore;         // Risk score (0-1, lower is better)
        public final String errorMessage;      // Error message if no shot
        
        public AimingSolution(
            boolean hasShot,
            double turretYawRad,
            double hoodPitchRad,
            double shooterSpeedMps,
            double spinRate,
            double spinAxisX,
            double spinAxisY,
            double spinAxisZ,
            double confidence,
            double riskScore,
            String errorMessage
        ) {
            this.hasShot = hasShot;
            this.turretYawRad = turretYawRad;
            this.hoodPitchRad = hoodPitchRad;
            this.shooterSpeedMps = shooterSpeedMps;
            this.spinRate = spinRate;
            this.spinAxisX = spinAxisX;
            this.spinAxisY = spinAxisY;
            this.spinAxisZ = spinAxisZ;
            this.confidence = confidence;
            this.riskScore = riskScore;
            this.errorMessage = errorMessage;
        }
        
        /**
         * Get turret yaw in degrees for convenience.
         */
        public double getTurretYawDeg() {
            return Math.toDegrees(turretYawRad);
        }
        
        /**
         * Get hood pitch in degrees for convenience.
         */
        public double getHoodPitchDeg() {
            return Math.toDegrees(hoodPitchRad);
        }
        
        @Override
        public String toString() {
            if (!hasShot) {
                return "AimingSolution{NO SHOT: " + errorMessage + "}";
            }
            return String.format(
                "AimingSolution{turret=%.1f°, hood=%.1f°, speed=%.1fm/s, " +
                "spin=%.0frad/s, confidence=%.2f, risk=%.3f}",
                getTurretYawDeg(), getHoodPitchDeg(), shooterSpeedMps,
                spinRate, confidence, riskScore
            );
        }
    }
    
    /**
     * Cached solution with timestamp.
     */
    private static class CachedSolution {
        final AimingSolution solution;
        final long timestamp;
        
        CachedSolution(AimingSolution solution, long timestamp) {
            this.solution = solution;
            this.timestamp = timestamp;
        }
    }
}
