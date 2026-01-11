package com.ruthiefrc.trajectory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Calibration system for learning from shot data.
 * Supports logging shots and fitting model parameters.
 */
public class CalibrationSystem {
    private final List<ShotLog> shotHistory;
    private CalibrationParameters currentParameters;
    
    public CalibrationSystem() {
        this.shotHistory = new ArrayList<>();
        this.currentParameters = new CalibrationParameters();
    }
    
    public CalibrationSystem(CalibrationParameters initialParameters) {
        this.shotHistory = new ArrayList<>();
        this.currentParameters = initialParameters;
    }
    
    /**
     * Log a shot for future calibration.
     */
    public void logShot(ShotLog shot) {
        shotHistory.add(shot);
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
     * Perform calibration using logged shot data.
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
