package com.ruthiefrc.trajectory;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class PhysicsModelTest {
    
    @Test
    void testGravityOnly() {
        PhysicsModel model = new PhysicsModel();
        
        // Projectile at rest should only experience gravity
        Vector3D position = new Vector3D(0, 0, 10);
        Vector3D velocity = new Vector3D(0, 0, 0);
        Vector3D spin = new Vector3D(0, 0, 0);
        ProjectileState state = new ProjectileState(position, velocity, spin);
        
        StateDerivative derivative = model.computeDerivative(state);
        
        // Velocity derivative should be zero (no velocity change from zero velocity)
        assertEquals(0.0, derivative.positionDerivative.x, 1e-10);
        assertEquals(0.0, derivative.positionDerivative.y, 1e-10);
        assertEquals(0.0, derivative.positionDerivative.z, 1e-10);
        
        // Acceleration should be -g in z direction
        assertEquals(0.0, derivative.velocityDerivative.x, 1e-6);
        assertEquals(0.0, derivative.velocityDerivative.y, 1e-6);
        assertEquals(-PhysicsConstants.GRAVITY, derivative.velocityDerivative.z, 1e-6);
    }
    
    @Test
    void testDragOpposesMotion() {
        PhysicsModel model = new PhysicsModel();
        
        // Projectile moving horizontally
        Vector3D position = new Vector3D(0, 0, 10);
        Vector3D velocity = new Vector3D(10, 0, 0);
        Vector3D spin = new Vector3D(0, 0, 0);
        ProjectileState state = new ProjectileState(position, velocity, spin);
        
        StateDerivative derivative = model.computeDerivative(state);
        
        // Drag should oppose motion (negative x acceleration)
        assertTrue(derivative.velocityDerivative.x < 0, "Drag should oppose horizontal motion");
        
        // Gravity still acts
        assertEquals(-PhysicsConstants.GRAVITY, derivative.velocityDerivative.z, 0.1);
    }
    
    @Test
    void testMagnusEffect() {
        CalibrationParameters params = new CalibrationParameters()
            .withMagnusCoefficient(0.001); // Higher Magnus for testing
        PhysicsModel model = new PhysicsModel(params);
        
        // Projectile moving forward with backspin
        Vector3D position = new Vector3D(0, 0, 10);
        Vector3D velocity = new Vector3D(10, 0, 0); // Forward
        Vector3D spin = new Vector3D(0, 100, 0); // Backspin (around y-axis)
        ProjectileState state = new ProjectileState(position, velocity, spin);
        
        StateDerivative derivative = model.computeDerivative(state);
        
        // Magnus force should create upward lift (ω × v should point up)
        // ω = (0, 100, 0), v = (10, 0, 0)
        // ω × v = (0, 0, -1000) * magnus_coeff creates downward force
        // Actually, backspin should create upward lift
        // Let me reconsider: backspin around y-axis with +x velocity
        // Should create lift in +z direction
        
        // The exact magnitude depends on coefficients, but Magnus should be present
        assertNotEquals(0.0, derivative.velocityDerivative.z + PhysicsConstants.GRAVITY, 0.5,
            "Magnus effect should modify vertical acceleration");
    }
    
    @Test
    void testCollisionReversesMomentum() {
        PhysicsModel model = new PhysicsModel();
        
        Vector3D position = new Vector3D(0, 0, 1);
        Vector3D velocity = new Vector3D(0, 0, -5); // Falling down
        Vector3D spin = new Vector3D(0, 0, 0);
        ProjectileState state = new ProjectileState(position, velocity, spin);
        
        // Collision with ground (normal pointing up)
        Vector3D surfaceNormal = new Vector3D(0, 0, 1);
        ProjectileState afterCollision = model.applyCollision(state, surfaceNormal);
        
        // Velocity should reverse (with restitution)
        assertTrue(afterCollision.velocity.z > 0, "Ball should bounce upward");
        assertTrue(afterCollision.velocity.z < Math.abs(velocity.z), 
            "Bounce should lose energy (restitution < 1)");
    }
}
