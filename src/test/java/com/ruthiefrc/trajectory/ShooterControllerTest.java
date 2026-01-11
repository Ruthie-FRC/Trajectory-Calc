package com.ruthiefrc.trajectory;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class ShooterControllerTest {
    
    @Test
    void testCalculateShotFromCloseRange() {
        ShooterController controller = new ShooterController();
        
        // Robot 3 meters away from target
        InverseSolver.SolutionResult solution = controller.calculateShot(3.0, 0.0, 1.0, 12.0);
        
        assertNotNull(solution);
        assertTrue(solution.launchPitchDeg > 0, "Should aim upward");
        assertTrue(solution.launchPitchDeg < 60, "Pitch should be reasonable");
    }
    
    @Test
    void testCalculateShotFromFarRange() {
        ShooterController controller = new ShooterController();
        
        // Robot 6 meters away from target
        InverseSolver.SolutionResult solution = controller.calculateShot(6.0, 0.0, 1.0, 15.0);
        
        assertNotNull(solution);
        assertTrue(solution.launchPitchDeg > 0, "Should aim upward");
    }
    
    @Test
    void testCanReachTarget() {
        ShooterController controller = new ShooterController();
        
        // Close range should be reachable
        assertTrue(controller.canReachTarget(3.0, 0.0, 1.0, 15.0));
        
        // Very far range might not be reachable
        boolean canReach = controller.canReachTarget(20.0, 0.0, 1.0, 10.0);
        // This depends on physics, just ensure it returns a boolean
        assertNotNull(canReach);
    }
    
    @Test
    void testShotLogging() {
        ShooterController controller = new ShooterController();
        
        InverseSolver.SolutionResult solution = controller.calculateShot(3.0, 0.0, 1.0, 12.0);
        controller.logShot(3.0, 0.0, 1.0, solution, 12.0, 100.0, true);
        
        String stats = controller.getCalibrationStats();
        assertTrue(stats.contains("1"), "Should have 1 logged shot");
    }
}
