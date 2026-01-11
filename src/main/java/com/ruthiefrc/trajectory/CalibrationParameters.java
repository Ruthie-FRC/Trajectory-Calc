package com.ruthiefrc.trajectory;

/**
 * Calibration parameters for the trajectory model.
 * These parameters can be tuned via nonlinear least-squares regression.
 * All parameters are validated against safety limits.
 */
public class CalibrationParameters {
    public final double dragCoefficient;
    public final double magnusCoefficient;
    public final double speedEfficiency;
    public final double spinEfficiency;
    public final double restitutionCoefficient;
    public final double frictionCoefficient;
    
    // Spin physics parameters (NEW)
    public final double spinDecayRate; // per second for foam ball spin decay
    
    // Per-shot correction factors
    public final double speedCorrectionFactor; // multiplicative correction per shot
    public final double spinCorrectionFactor; // multiplicative correction per shot
    
    /**
     * Create calibration parameters with default values.
     */
    public CalibrationParameters() {
        this(PhysicsConstants.DEFAULT_DRAG_COEFFICIENT,
             PhysicsConstants.DEFAULT_MAGNUS_COEFFICIENT,
             PhysicsConstants.DEFAULT_SPEED_EFFICIENCY,
             PhysicsConstants.DEFAULT_SPIN_EFFICIENCY,
             PhysicsConstants.DEFAULT_RESTITUTION_COEFFICIENT,
             PhysicsConstants.DEFAULT_FRICTION_COEFFICIENT,
             PhysicsConstants.DEFAULT_SPIN_DECAY_RATE,
             1.0, // no speed correction by default
             1.0); // no spin correction by default
    }
    
    /**
     * Create calibration parameters with custom values (legacy constructor).
     */
    public CalibrationParameters(double dragCoefficient, double magnusCoefficient,
                                  double speedEfficiency, double spinEfficiency,
                                  double restitutionCoefficient, double frictionCoefficient) {
        this(dragCoefficient, magnusCoefficient, speedEfficiency, spinEfficiency,
             restitutionCoefficient, frictionCoefficient, 
             PhysicsConstants.DEFAULT_SPIN_DECAY_RATE, 1.0, 1.0);
    }
    
    /**
     * Create calibration parameters with custom values including spin decay and correction factors.
     */
    public CalibrationParameters(double dragCoefficient, double magnusCoefficient,
                                  double speedEfficiency, double spinEfficiency,
                                  double restitutionCoefficient, double frictionCoefficient,
                                  double spinDecayRate,
                                  double speedCorrectionFactor, double spinCorrectionFactor) {
        // Validate and clamp parameters to safety limits
        this.dragCoefficient = clamp(dragCoefficient, 
            PhysicsConstants.MIN_DRAG_COEFFICIENT, PhysicsConstants.MAX_DRAG_COEFFICIENT);
        this.magnusCoefficient = clamp(magnusCoefficient,
            PhysicsConstants.MIN_MAGNUS_COEFFICIENT, PhysicsConstants.MAX_MAGNUS_COEFFICIENT);
        this.speedEfficiency = clamp(speedEfficiency,
            PhysicsConstants.MIN_SPEED_EFFICIENCY, PhysicsConstants.MAX_SPEED_EFFICIENCY);
        this.spinEfficiency = clamp(spinEfficiency,
            PhysicsConstants.MIN_SPIN_EFFICIENCY, PhysicsConstants.MAX_SPIN_EFFICIENCY);
        this.restitutionCoefficient = clamp(restitutionCoefficient,
            PhysicsConstants.MIN_RESTITUTION, PhysicsConstants.MAX_RESTITUTION);
        this.frictionCoefficient = clamp(frictionCoefficient,
            PhysicsConstants.MIN_FRICTION, PhysicsConstants.MAX_FRICTION);
        this.spinDecayRate = clamp(spinDecayRate,
            PhysicsConstants.MIN_SPIN_DECAY_RATE, PhysicsConstants.MAX_SPIN_DECAY_RATE);
        this.speedCorrectionFactor = clamp(speedCorrectionFactor, 0.8, 1.2);
        this.spinCorrectionFactor = clamp(spinCorrectionFactor, 0.8, 1.2);
    }
    
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    public CalibrationParameters withDragCoefficient(double value) {
        return new CalibrationParameters(value, magnusCoefficient, speedEfficiency,
            spinEfficiency, restitutionCoefficient, frictionCoefficient, spinDecayRate,
            speedCorrectionFactor, spinCorrectionFactor);
    }
    
    public CalibrationParameters withMagnusCoefficient(double value) {
        return new CalibrationParameters(dragCoefficient, value, speedEfficiency,
            spinEfficiency, restitutionCoefficient, frictionCoefficient, spinDecayRate,
            speedCorrectionFactor, spinCorrectionFactor);
    }
    
    public CalibrationParameters withSpeedEfficiency(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, value,
            spinEfficiency, restitutionCoefficient, frictionCoefficient, spinDecayRate,
            speedCorrectionFactor, spinCorrectionFactor);
    }
    
    public CalibrationParameters withSpinEfficiency(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            value, restitutionCoefficient, frictionCoefficient, spinDecayRate,
            speedCorrectionFactor, spinCorrectionFactor);
    }
    
    public CalibrationParameters withRestitutionCoefficient(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            spinEfficiency, value, frictionCoefficient, spinDecayRate,
            speedCorrectionFactor, spinCorrectionFactor);
    }
    
    public CalibrationParameters withFrictionCoefficient(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            spinEfficiency, restitutionCoefficient, value, spinDecayRate,
            speedCorrectionFactor, spinCorrectionFactor);
    }
    
    public CalibrationParameters withSpinDecayRate(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            spinEfficiency, restitutionCoefficient, frictionCoefficient, value,
            speedCorrectionFactor, spinCorrectionFactor);
    }
    
    public CalibrationParameters withSpeedCorrectionFactor(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            spinEfficiency, restitutionCoefficient, frictionCoefficient, spinDecayRate,
            value, spinCorrectionFactor);
    }
    
    public CalibrationParameters withSpinCorrectionFactor(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            spinEfficiency, restitutionCoefficient, frictionCoefficient, spinDecayRate,
            speedCorrectionFactor, value);
    }
    
    /**
     * Check if parameters are within safe operational ranges.
     */
    public boolean isValid() {
        return dragCoefficient >= PhysicsConstants.MIN_DRAG_COEFFICIENT &&
               dragCoefficient <= PhysicsConstants.MAX_DRAG_COEFFICIENT &&
               speedEfficiency >= PhysicsConstants.MIN_SPEED_EFFICIENCY &&
               speedEfficiency <= PhysicsConstants.MAX_SPEED_EFFICIENCY &&
               spinEfficiency >= PhysicsConstants.MIN_SPIN_EFFICIENCY &&
               spinEfficiency <= PhysicsConstants.MAX_SPIN_EFFICIENCY;
    }
    
    @Override
    public String toString() {
        return String.format("CalibrationParameters[Cd=%.3f, Cm=%.6f, ηv=%.3f, ηω=%.3f, e=%.2f, μ=%.2f, decay=%.3f, Δv=%.3f, Δω=%.3f]",
            dragCoefficient, magnusCoefficient, speedEfficiency, spinEfficiency,
            restitutionCoefficient, frictionCoefficient, spinDecayRate, speedCorrectionFactor, spinCorrectionFactor);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof CalibrationParameters)) return false;
        CalibrationParameters other = (CalibrationParameters) obj;
        return Double.compare(dragCoefficient, other.dragCoefficient) == 0 &&
               Double.compare(magnusCoefficient, other.magnusCoefficient) == 0 &&
               Double.compare(speedEfficiency, other.speedEfficiency) == 0 &&
               Double.compare(spinEfficiency, other.spinEfficiency) == 0 &&
               Double.compare(restitutionCoefficient, other.restitutionCoefficient) == 0 &&
               Double.compare(frictionCoefficient, other.frictionCoefficient) == 0 &&
               Double.compare(spinDecayRate, other.spinDecayRate) == 0 &&
               Double.compare(speedCorrectionFactor, other.speedCorrectionFactor) == 0 &&
               Double.compare(spinCorrectionFactor, other.spinCorrectionFactor) == 0;
    }
    
    @Override
    public int hashCode() {
        long bits = Double.doubleToLongBits(dragCoefficient);
        bits ^= Double.doubleToLongBits(magnusCoefficient) * 31;
        bits ^= Double.doubleToLongBits(speedEfficiency) * 37;
        bits ^= Double.doubleToLongBits(spinEfficiency) * 41;
        return (int) (bits ^ (bits >>> 32));
    }
}
