package com.ruthiefrc.trajectory.empirical;

import com.ruthiefrc.trajectory.Vector3D;

/**
 * Represents a single recorded shot attempt with inputs and outcome.
 * 
 * <p>Immutable data class for storing shot telemetry.
 */
public class ShotRecord {
    
    // Input parameters
    public final double robotX;
    public final double robotY;
    public final double robotHeading;
    public final double turretYaw;
    public final double hoodPitch;
    public final double shooterVelocity;
    public final double spinRate;
    public final Vector3D spinAxis;
    public final double distanceToTarget;
    public final long timestamp;
    
    // Outcome
    public final ShotOutcome outcome;
    public final RimLocation rimLocation;
    public final double lateralError;
    public final double verticalError;
    
    public ShotRecord(
            double robotX, double robotY, double robotHeading,
            double turretYaw, double hoodPitch,
            double shooterVelocity, double spinRate, Vector3D spinAxis,
            double distanceToTarget, long timestamp,
            ShotOutcome outcome, RimLocation rimLocation,
            double lateralError, double verticalError) {
        
        this.robotX = robotX;
        this.robotY = robotY;
        this.robotHeading = robotHeading;
        this.turretYaw = turretYaw;
        this.hoodPitch = hoodPitch;
        this.shooterVelocity = shooterVelocity;
        this.spinRate = spinRate;
        this.spinAxis = spinAxis;
        this.distanceToTarget = distanceToTarget;
        this.timestamp = timestamp;
        this.outcome = outcome;
        this.rimLocation = rimLocation;
        this.lateralError = lateralError;
        this.verticalError = verticalError;
    }
    
    @Override
    public String toString() {
        return String.format("ShotRecord[dist=%.2fm, outcome=%s, rim=%s, error=(%.3f, %.3f)]",
                distanceToTarget, outcome, rimLocation, lateralError, verticalError);
    }
    
    /**
     * Builder for constructing ShotRecord instances.
     */
    public static class Builder {
        private double robotX, robotY, robotHeading;
        private double turretYaw, hoodPitch;
        private double shooterVelocity, spinRate;
        private double spinAxisX, spinAxisY, spinAxisZ;
        private double distanceToTarget;
        private long timestamp;
        private ShotOutcome outcome;
        private RimLocation rimLocation;
        private double lateralError, verticalError;
        
        public Builder robotX(double v) { robotX = v; return this; }
        public Builder robotY(double v) { robotY = v; return this; }
        public Builder robotHeading(double v) { robotHeading = v; return this; }
        public Builder turretYaw(double v) { turretYaw = v; return this; }
        public Builder hoodPitch(double v) { hoodPitch = v; return this; }
        public Builder shooterVelocity(double v) { shooterVelocity = v; return this; }
        public Builder spinRate(double v) { spinRate = v; return this; }
        public Builder spinAxisX(double v) { spinAxisX = v; return this; }
        public Builder spinAxisY(double v) { spinAxisY = v; return this; }
        public Builder spinAxisZ(double v) { spinAxisZ = v; return this; }
        public Builder distanceToTarget(double v) { distanceToTarget = v; return this; }
        public Builder timestamp(long v) { timestamp = v; return this; }
        public Builder outcome(ShotOutcome v) { outcome = v; return this; }
        public Builder rimLocation(RimLocation v) { rimLocation = v; return this; }
        public Builder lateralError(double v) { lateralError = v; return this; }
        public Builder verticalError(double v) { verticalError = v; return this; }
        
        public ShotRecord build() {
            return new ShotRecord(
                    robotX, robotY, robotHeading,
                    turretYaw, hoodPitch,
                    shooterVelocity, spinRate,
                    new Vector3D(spinAxisX, spinAxisY, spinAxisZ),
                    distanceToTarget, timestamp,
                    outcome, rimLocation,
                    lateralError, verticalError
            );
        }
    }
}
