package com.ruthiefrc.trajectory;

/**
 * Calibration parameters for the trajectory model.
 * These parameters can be tuned via nonlinear least-squares regression.
 */
public class CalibrationParameters {
    public final double dragCoefficient;
    public final double magnusCoefficient;
    public final double speedEfficiency;
    public final double spinEfficiency;
    public final double restitutionCoefficient;
    public final double frictionCoefficient;
    
    /**
     * Create calibration parameters with default values.
     */
    public CalibrationParameters() {
        this(PhysicsConstants.DRAG_COEFFICIENT,
             PhysicsConstants.MAGNUS_COEFFICIENT,
             PhysicsConstants.SPEED_EFFICIENCY,
             PhysicsConstants.SPIN_EFFICIENCY,
             PhysicsConstants.RESTITUTION_COEFFICIENT,
             PhysicsConstants.FRICTION_COEFFICIENT);
    }
    
    /**
     * Create calibration parameters with custom values.
     */
    public CalibrationParameters(double dragCoefficient, double magnusCoefficient,
                                  double speedEfficiency, double spinEfficiency,
                                  double restitutionCoefficient, double frictionCoefficient) {
        this.dragCoefficient = dragCoefficient;
        this.magnusCoefficient = magnusCoefficient;
        this.speedEfficiency = speedEfficiency;
        this.spinEfficiency = spinEfficiency;
        this.restitutionCoefficient = restitutionCoefficient;
        this.frictionCoefficient = frictionCoefficient;
    }
    
    public CalibrationParameters withDragCoefficient(double value) {
        return new CalibrationParameters(value, magnusCoefficient, speedEfficiency,
            spinEfficiency, restitutionCoefficient, frictionCoefficient);
    }
    
    public CalibrationParameters withMagnusCoefficient(double value) {
        return new CalibrationParameters(dragCoefficient, value, speedEfficiency,
            spinEfficiency, restitutionCoefficient, frictionCoefficient);
    }
    
    public CalibrationParameters withSpeedEfficiency(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, value,
            spinEfficiency, restitutionCoefficient, frictionCoefficient);
    }
    
    public CalibrationParameters withSpinEfficiency(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            value, restitutionCoefficient, frictionCoefficient);
    }
    
    public CalibrationParameters withRestitutionCoefficient(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            spinEfficiency, value, frictionCoefficient);
    }
    
    public CalibrationParameters withFrictionCoefficient(double value) {
        return new CalibrationParameters(dragCoefficient, magnusCoefficient, speedEfficiency,
            spinEfficiency, restitutionCoefficient, value);
    }
    
    @Override
    public String toString() {
        return String.format("CalibrationParameters[Cd=%.3f, Cm=%.6f, ηv=%.3f, ηω=%.3f, e=%.2f, μ=%.2f]",
            dragCoefficient, magnusCoefficient, speedEfficiency, spinEfficiency,
            restitutionCoefficient, frictionCoefficient);
    }
}
