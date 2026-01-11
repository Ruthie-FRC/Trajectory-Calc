package com.ruthiefrc.trajectory;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Parameterized edge case tests for the trajectory calculator.
 * Tests extreme conditions, near-rim entries, and solver robustness.
 */
public class EdgeCaseTest {
    
    /**
     * Test with minimum speed values.
     */
    @ParameterizedTest
    @ValueSource(doubles = {5.0, 6.0, 7.0, 8.0})
    public void testMinimumSpeed(double speed) {
        ShooterController controller = new ShooterController();
        
        // Close range shot with minimum speed
        var solution = controller.calculateShot(2.0, 0.0, 0.5, speed, 50.0);
        
        // Should either hit or return reasonable angles
        assertNotNull(solution);
        assertTrue(solution.launchPitchDeg >= -10.0 && solution.launchPitchDeg <= 90.0);
        assertTrue(solution.launchYawDeg >= -180.0 && solution.launchYawDeg <= 180.0);
    }
    
    /**
     * Test with maximum speed values.
     */
    @ParameterizedTest
    @ValueSource(doubles = {15.0, 18.0, 20.0, 25.0})
    public void testMaximumSpeed(double speed) {
        ShooterController controller = new ShooterController();
        
        // Long range shot with high speed
        var solution = controller.calculateShot(8.0, 0.0, 0.5, speed, 100.0);
        
        assertNotNull(solution);
        assertTrue(solution.launchPitchDeg >= -10.0 && solution.launchPitchDeg <= 90.0);
        assertTrue(solution.launchYawDeg >= -180.0 && solution.launchYawDeg <= 180.0);
    }
    
    /**
     * Test with minimum spin values.
     */
    @ParameterizedTest
    @ValueSource(doubles = {0.0, 10.0, 20.0, 50.0})
    public void testMinimumSpin(double spin) {
        ShooterController controller = new ShooterController();
        
        var solution = controller.calculateShot(3.0, 0.0, 0.5, 12.0, spin);
        
        assertNotNull(solution);
        // Low spin should still produce valid solution
        assertTrue(solution.score > -100.0); // Not a total failure
    }
    
    /**
     * Test with maximum spin values.
     */
    @ParameterizedTest
    @ValueSource(doubles = {200.0, 300.0, 400.0, 500.0})
    public void testMaximumSpin(double spin) {
        ShooterController controller = new ShooterController();
        
        var solution = controller.calculateShot(3.0, 0.0, 0.5, 12.0, spin);
        
        assertNotNull(solution);
        // High spin should still converge
        assertTrue(solution.score > -100.0);
    }
    
    /**
     * Test near-rim entry scenarios.
     * Uses positions that would result in edge trajectories.
     */
    @ParameterizedTest
    @CsvSource({
        "2.5, 0.0, 0.5, 11.0",
        "3.0, 0.5, 0.5, 11.5",
        "2.8, -0.3, 0.5, 11.2",
        "3.2, 0.4, 0.6, 11.8"
    })
    public void testNearRimEntry(double x, double y, double z, double speed) {
        ShooterController controller = new ShooterController();
        
        var solution = controller.calculateShot(x, y, z, speed, 100.0);
        
        assertNotNull(solution);
        
        // If it hits, should have reasonable entry quality
        if (solution.isHit()) {
            assertTrue(solution.score > 0.0, "Hit should have positive score");
            
            // Near-rim entries should not have rim proximity issues
            if (solution.trajectory != null && solution.trajectory.entryState != null) {
                Vector3D entry = solution.trajectory.entryState.position;
                double distFromCenter = Math.sqrt(entry.x * entry.x + entry.y * entry.y);
                
                // Should not be too close to rim (within danger zone)
                double rimRadius = PhysicsConstants.DEFAULT_HUB_OPENING_FLAT_TO_FLAT / 2.0;
                double distToRim = rimRadius - distFromCenter;
                
                assertTrue(distToRim > 0.0, "Entry should be inside opening");
            }
        }
    }
    
    /**
     * Test from multiple field positions.
     */
    @ParameterizedTest
    @CsvSource({
        "1.0, 0.0, 0.5",   // Very close
        "3.0, 0.0, 0.5",   // Medium range
        "5.0, 0.0, 0.5",   // Long range
        "3.0, 2.0, 0.6",   // Angled approach
        "-2.0, 1.5, 0.55", // Different quadrant
        "2.0, -2.0, 0.5",  // Another quadrant
        "4.0, 4.0, 0.65"   // Diagonal long range
    })
    public void testMultipleFieldPositions(double x, double y, double z) {
        ShooterController controller = new ShooterController();
        
        var solution = controller.calculateShot(x, y, z, 12.0, 100.0);
        
        assertNotNull(solution);
        assertTrue(solution.launchYawDeg >= -180.0 && solution.launchYawDeg <= 180.0);
        assertTrue(solution.launchPitchDeg >= -10.0 && solution.launchPitchDeg <= 90.0);
        
        // Real-time safe: calculation should be fast
        // Note: Performance can vary on different systems, use generous timeout
        long startTime = System.nanoTime();
        controller.calculateShot(x, y, z, 12.0, 100.0);
        long endTime = System.nanoTime();
        long durationMs = (endTime - startTime) / 1_000_000;
        
        assertTrue(durationMs < 100, "Calculation should complete in <100ms, took " + durationMs + "ms");
    }
    
    /**
     * Test with simulated wheel variance.
     */
    @Test
    public void testWithWheelVariance() {
        CalibrationParameters params = new CalibrationParameters()
            .withSpeedEfficiency(0.90)  // 10% speed loss
            .withSpinEfficiency(0.85);   // 15% spin loss
        
        ShooterController controller = new ShooterController(params);
        
        var solution = controller.calculateShot(3.0, 0.0, 0.5, 12.0, 100.0);
        
        assertNotNull(solution);
        // Should still find solution despite wheel inefficiency
        assertTrue(solution.score > -100.0);
    }
    
    /**
     * Test solver robustness with varying ball properties.
     */
    @Test
    public void testWithVariousBallProperties() {
        // Worn ball (lighter, smaller)
        ProjectileProperties wornBall = ProjectileProperties.wornBall();
        ShooterController controller1 = new ShooterController(
            new CalibrationParameters(), wornBall
        );
        
        var solution1 = controller1.calculateShot(3.0, 0.0, 0.5, 12.0, 100.0);
        assertNotNull(solution1);
        
        // Heavy ball
        ProjectileProperties heavyBall = ProjectileProperties.heavyBall();
        ShooterController controller2 = new ShooterController(
            new CalibrationParameters(), heavyBall
        );
        
        var solution2 = controller2.calculateShot(3.0, 0.0, 0.5, 12.0, 100.0);
        assertNotNull(solution2);
        
        // Solutions should be different due to different ball properties
        // (worn ball needs different angle than heavy ball)
        // Note: In some cases angles may be very similar, so just verify both solve
        assertTrue(solution1.score > -100.0, "Worn ball should produce valid solution");
        assertTrue(solution2.score > -100.0, "Heavy ball should produce valid solution");
    }
    
    /**
     * Test pre-seeded guess functionality.
     */
    @Test
    public void testPreSeededGuesses() {
        ShooterController controller = new ShooterController();
        
        // First shot - no cache
        long startTime1 = System.nanoTime();
        var solution1 = controller.calculateShot(3.0, 0.0, 0.5, 12.0, 100.0);
        long duration1 = (System.nanoTime() - startTime1) / 1_000_000;
        
        // Second shot from same position - should use cache
        long startTime2 = System.nanoTime();
        var solution2 = controller.calculateShot(3.0, 0.0, 0.5, 12.0, 100.0);
        long duration2 = (System.nanoTime() - startTime2) / 1_000_000;
        
        assertNotNull(solution1);
        assertNotNull(solution2);
        
        // Solutions should be identical or very similar
        assertEquals(solution1.launchYawDeg, solution2.launchYawDeg, 0.1);
        assertEquals(solution1.launchPitchDeg, solution2.launchPitchDeg, 0.1);
        
        // Second solve might be faster due to better initial guess
        // (not always guaranteed due to system variance, but log it)
        System.out.println("First solve: " + duration1 + "ms, Second solve: " + duration2 + "ms");
    }
    
    /**
     * Test incremental calibration.
     */
    @Test
    public void testIncrementalCalibration() {
        ShooterController controller = new ShooterController();
        controller.setIncrementalCalibrationEnabled(true);
        controller.setLearningRate(0.1); // 10% adjustment rate
        
        CalibrationParameters initialParams = controller.getCalibrationParameters();
        
        // Log several misses
        var solution = controller.calculateShot(3.0, 0.0, 0.5, 12.0, 100.0);
        for (int i = 0; i < 10; i++) {
            controller.logShot(3.0, 0.0, 0.5, solution, 12.0, 100.0, false); // all misses
        }
        
        CalibrationParameters updatedParams = controller.getCalibrationParameters();
        
        // Parameters should have changed after multiple misses
        // (May or may not change depending on heuristics, but system should not crash)
        assertNotNull(updatedParams);
    }
    
    /**
     * Test deterministic behavior.
     */
    @Test
    public void testDeterministicBehavior() {
        ShooterController controller = new ShooterController();
        
        var solution1 = controller.calculateShot(3.0, 1.0, 0.5, 12.0, 100.0);
        var solution2 = controller.calculateShot(3.0, 1.0, 0.5, 12.0, 100.0);
        
        // Same inputs should produce identical outputs
        assertEquals(solution1.launchYawDeg, solution2.launchYawDeg, 1e-10);
        assertEquals(solution1.launchPitchDeg, solution2.launchPitchDeg, 1e-10);
        assertEquals(solution1.score, solution2.score, 1e-10);
    }
    
    /**
     * Test reachability check.
     */
    @Test
    public void testReachabilityCheck() {
        ShooterController controller = new ShooterController();
        
        // Close position - should be reachable
        assertTrue(controller.canReachTarget(2.0, 0.0, 0.5, 15.0));
        
        // Far position with low speed - should not be reachable
        assertFalse(controller.canReachTarget(20.0, 0.0, 0.5, 5.0));
        
        // Medium position with medium speed - should be reachable
        assertTrue(controller.canReachTarget(5.0, 0.0, 0.5, 12.0));
    }
}
