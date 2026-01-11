package com.ruthiefrc.trajectory;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class TrajectorySimulatorTest {
    
    @Test
    void testSimpleParabolicTrajectory() {
        PhysicsModel model = new PhysicsModel();
        TrajectorySimulator simulator = new TrajectorySimulator(model, new HubGeometry());
        
        // Launch projectile at 45 degrees
        Vector3D position = new Vector3D(0, 0, 1);
        double speed = 10.0;
        double angle = Math.toRadians(45);
        Vector3D velocity = new Vector3D(speed * Math.cos(angle), 0, speed * Math.sin(angle));
        Vector3D spin = new Vector3D(0, 0, 0);
        
        ProjectileState initialState = new ProjectileState(position, velocity, spin);
        TrajectorySimulator.TrajectoryResult result = simulator.simulate(initialState);
        
        assertNotNull(result);
        assertFalse(result.trajectory.isEmpty());
        assertTrue(result.getMaxHeight() > 1.0, "Projectile should reach significant height");
        assertTrue(result.getFlightTime() > 0.5, "Flight should take measurable time");
    }
    
    @Test
    void testProjectileFallsDown() {
        PhysicsModel model = new PhysicsModel();
        TrajectorySimulator simulator = new TrajectorySimulator(model, new HubGeometry());
        
        // Launch projectile upward
        Vector3D position = new Vector3D(0, 0, 2);
        Vector3D velocity = new Vector3D(0, 0, 5);
        Vector3D spin = new Vector3D(0, 0, 0);
        
        ProjectileState initialState = new ProjectileState(position, velocity, spin);
        TrajectorySimulator.TrajectoryResult result = simulator.simulate(initialState);
        
        // Final position should be at or below ground
        ProjectileState finalState = result.trajectory.get(result.trajectory.size() - 1);
        assertTrue(finalState.position.z <= 0.1, "Projectile should return to ground");
    }
    
    @Test
    void testShooterModelAppliesEfficiency() {
        CalibrationParameters params = new CalibrationParameters()
            .withSpeedEfficiency(0.9);
        PhysicsModel model = new PhysicsModel(params);
        TrajectorySimulator simulator = new TrajectorySimulator(model, new HubGeometry());
        
        Vector3D robotPos = new Vector3D(0, 0, 1);
        double commandedSpeed = 10.0;
        
        TrajectorySimulator.TrajectoryResult result = simulator.simulateWithShooterModel(
            robotPos, commandedSpeed, 0, 45, 0
        );
        
        // Initial velocity magnitude should be less than commanded (due to efficiency)
        ProjectileState initialState = result.trajectory.get(0);
        double actualSpeed = initialState.velocity.magnitude();
        assertTrue(actualSpeed < commandedSpeed, "Actual speed should be less due to efficiency");
        assertTrue(actualSpeed > commandedSpeed * 0.85, "Speed should be close to commanded * efficiency");
    }
}
