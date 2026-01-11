package frc.robot.trajectory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Calibration system for learning from shot data.
 * Supports logging shots and fitting model parameters.
 * Enhanced with incremental regression for per-shot parameter updates.
 */
public class CalibrationSystem {
    private final List<ShotLog> shotHistory;
    private CalibrationParameters currentParameters;
    private boolean incrementalCalibrationEnabled;
    
    // Incremental learning parameters
    private double learningRate;
    private int minShotsForIncremental;
    
    public CalibrationSystem() {
        this.shotHistory = new ArrayList<>();
        this.currentParameters = new CalibrationParameters();
        this.incrementalCalibrationEnabled = false; // Off by default
        this.learningRate = 0.05; // 5% adjustment per shot
        this.minShotsForIncremental = 5; // Need baseline before incremental updates
    }
    
    public CalibrationSystem(CalibrationParameters initialParameters) {
        this.shotHistory = new ArrayList<>();
        this.currentParameters = initialParameters;
        this.incrementalCalibrationEnabled = false;
        this.learningRate = 0.05;
        this.minShotsForIncremental = 5;
    }
    
    /**
     * Enable or disable incremental calibration.
     * When enabled, parameters are updated after each shot.
     * Use during practice; disable during competition.
     */
    public void setIncrementalCalibrationEnabled(boolean enabled) {
        this.incrementalCalibrationEnabled = enabled;
    }
    
    public boolean isIncrementalCalibrationEnabled() {
        return incrementalCalibrationEnabled;
    }
    
    /**
     * Set learning rate for incremental updates (0.0 - 1.0).
     * Higher values = faster adaptation but less stability.
     * Default: 0.05 (5% per shot)
     */
    public void setLearningRate(double rate) {
        this.learningRate = Math.max(0.0, Math.min(1.0, rate));
    }
    
    public double getLearningRate() {
        return learningRate;
    }
    
    /**
     * Log a shot for future calibration.
     * If incremental calibration is enabled, updates parameters immediately.
     */
    public void logShot(ShotLog shot) {
        shotHistory.add(shot);
        
        // Incremental update if enabled and we have enough baseline data
        if (incrementalCalibrationEnabled && shotHistory.size() >= minShotsForIncremental) {
            performIncrementalUpdate(shot);
        }
    }
    
    /**
     * Log a shot from solution result.
     */
    public void logShot(Vector3D robotPose, InverseSolver.SolutionResult solution,
                       double launchSpeed, double spinRate, boolean actualHit) {
        ShotLog log = new ShotLog(robotPose, solution.launchYawDeg, solution.launchPitchDeg,
                                  launchSpeed, spinRate, actualHit);
        logShot(log);
    }
    
    /**
     * Perform incremental parameter update based on a single shot.
     * Uses online learning to adjust parameters without rebuilding model.
     * Enhanced to calibrate spin-related parameters.
     */
    private void performIncrementalUpdate(ShotLog shot) {
        // Get recent performance to determine if adjustment is needed
        int windowSize = Math.min(5, shotHistory.size());
        List<ShotLog> recentShots = shotHistory.subList(
            shotHistory.size() - windowSize, shotHistory.size()
        );
        
        long recentHits = recentShots.stream().filter(s -> s.hit).count();
        double recentHitRate = (double) recentHits / windowSize;
        
        // Only adjust if performance indicates a systematic issue
        if (recentHitRate < 0.6) { // Less than 60% hit rate
            // Incremental adjustment based on shot characteristics
            CalibrationParameters newParams = currentParameters;
            
            // Adjust drag based on shot distance and speed
            double distance = shot.robotPose.magnitude();
            if (distance > 5.0 && shot.launchSpeed > 12.0) {
                // Long-distance high-speed miss - likely drag coefficient issue
                double dragAdjustment = shot.hit ? 1.0 : (1.0 - learningRate * 0.5);
                newParams = newParams.withDragCoefficient(
                    currentParameters.dragCoefficient * dragAdjustment
                );
            }
            
            // Adjust speed efficiency based on consistent misses
            if (windowSize >= 5 && recentHits == 0) {
                // Multiple misses in a row - likely systematic speed error
                double speedAdjustment = 1.0 + learningRate;
                newParams = newParams.withSpeedEfficiency(
                    currentParameters.speedEfficiency * speedAdjustment
                );
            }
            
            // Adjust spin efficiency based on spin rate and hit rate
            if (shot.spinRate > 100.0) {
                // High spin shots missing - might need spin efficiency adjustment
                double avgSpinRecent = recentShots.stream()
                    .mapToDouble(s -> s.spinRate).average().orElse(0.0);
                if (avgSpinRecent > 100.0 && recentHits <= 2) {
                    // Multiple high-spin misses - adjust spin efficiency
                    double spinAdjustment = 1.0 - learningRate * 0.3;
                    newParams = newParams.withSpinEfficiency(
                        currentParameters.spinEfficiency * spinAdjustment
                    );
                }
            }
            
            // Only update if parameters changed meaningfully
            if (!newParams.equals(currentParameters)) {
                currentParameters = newParams;
            }
        } else if (recentHitRate > 0.8 && shot.hit) {
            // Good performance - small refinement towards current successful parameters
            // This helps stabilize around good values
            // No change needed - parameters are working well
        }
    }
    
    /**
     * Get all logged shots.
     */
    public List<ShotLog> getShotHistory() {
        return Collections.unmodifiableList(shotHistory);
    }
    
    /**
     * Get the number of logged shots.
     */
    public int getShotCount() {
        return shotHistory.size();
    }
    
    /**
     * Get hit rate from logged shots.
     */
    public double getHitRate() {
        if (shotHistory.isEmpty()) {
            return 0.0;
        }
        long hits = shotHistory.stream().filter(s -> s.hit).count();
        return (double) hits / shotHistory.size();
    }
    
    /**
     * Perform batch calibration using all logged shot data.
     * This is a simplified calibration - a full implementation would use
     * nonlinear least-squares regression (e.g., Levenberg-Marquardt).
     * 
     * For now, this adjusts parameters based on hit rate trends.
     */
    public CalibrationParameters calibrate() {
        if (shotHistory.size() < 10) {
            // Not enough data for calibration
            return currentParameters;
        }
        
        // Simple heuristic calibration based on recent performance
        int recentWindow = Math.min(20, shotHistory.size());
        List<ShotLog> recentShots = shotHistory.subList(shotHistory.size() - recentWindow, shotHistory.size());
        
        long recentHits = recentShots.stream().filter(s -> s.hit).count();
        double recentHitRate = (double) recentHits / recentWindow;
        
        // If hit rate is low, we might need to adjust parameters
        CalibrationParameters newParams = currentParameters;
        
        if (recentHitRate < 0.5) {
            // Adjust drag coefficient slightly (shots may be falling short or long)
            double avgSpeed = recentShots.stream().mapToDouble(s -> s.launchSpeed).average().orElse(10.0);
            if (avgSpeed > 12.0) {
                // High speed shots missing - might have too much drag
                newParams = newParams.withDragCoefficient(currentParameters.dragCoefficient * 0.98);
            } else {
                // Low speed shots missing - might have too little drag
                newParams = newParams.withDragCoefficient(currentParameters.dragCoefficient * 1.02);
            }
        }
        
        currentParameters = newParams;
        return currentParameters;
    }
    
    /**
     * Update calibration parameters manually.
     */
    public void updateParameters(CalibrationParameters parameters) {
        this.currentParameters = parameters;
    }
    
    /**
     * Get current calibration parameters.
     */
    public CalibrationParameters getCurrentParameters() {
        return currentParameters;
    }
    
    /**
     * Clear shot history.
     */
    public void clearHistory() {
        shotHistory.clear();
    }
}
