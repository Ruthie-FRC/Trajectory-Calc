package com.ruthiefrc.trajectory;

/**
 * Derivative of the projectile state (velocity, acceleration, spin rate of change).
 */
public class StateDerivative {
    public final Vector3D positionDerivative;  // velocity
    public final Vector3D velocityDerivative;  // acceleration
    public final Vector3D spinDerivative;      // angular acceleration
    
    public StateDerivative(Vector3D positionDerivative, Vector3D velocityDerivative, Vector3D spinDerivative) {
        this.positionDerivative = positionDerivative;
        this.velocityDerivative = velocityDerivative;
        this.spinDerivative = spinDerivative;
    }
    
    public StateDerivative scale(double scalar) {
        return new StateDerivative(
            positionDerivative.scale(scalar),
            velocityDerivative.scale(scalar),
            spinDerivative.scale(scalar)
        );
    }
    
    public StateDerivative add(StateDerivative other) {
        return new StateDerivative(
            positionDerivative.add(other.positionDerivative),
            velocityDerivative.add(other.velocityDerivative),
            spinDerivative.add(other.spinDerivative)
        );
    }
}
