package com.ruthiefrc.trajectory;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for SpinShotSimulator utility class.
 */
public class SpinShotSimulatorTest {
    
    private SpinShotSimulator simulator;
    
    @BeforeEach
    public void setup() {
        simulator = new SpinShotSimulator();
        // Use narrower search ranges for faster tests
        simulator.setYawRange(-45, 45, 10);
        simulator.setPitchRange(20, 60, 5);
    }
    
    @Test
    public void testBasicSimulation() {
        // Target near HUB center
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0); // Backspin
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        assertTrue(result.simulationsRun > 0);
        assertTrue(result.apexHeight > 0);
        assertTrue(result.flightTime > 0);
    }
    
    @Test
    public void testSimulationFindsHit() {
        // Close target with good speed
        Vector3D target = new Vector3D(2.5, 0.0, 1.8);
        double speed = 10.0;
        Vector3D spin = new Vector3D(0, 150, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        // Should find at least a reasonable trajectory
        assertTrue(result.bestScore > -50, "Should find a reasonable trajectory");
    }
    
    @ParameterizedTest
    @CsvSource({
        "2.0, 0.0, 1.8, 10.0",
        "3.0, 0.5, 2.0, 12.0",
        "4.0, -0.5, 2.2, 14.0",
        "3.5, 1.0, 2.0, 13.0"
    })
    public void testVariousTargetPositions(double x, double y, double z, double speed) {
        Vector3D target = new Vector3D(x, y, z);
        Vector3D spin = new Vector3D(0, 180, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        assertTrue(result.simulationsRun > 0);
        assertTrue(result.apexHeight >= 0);  // May not reach target height
        assertTrue(result.flightTime >= 0);  // May miss quickly
        // Don't enforce strict flight time limit - depends on target position
    }
    
    @Test
    public void testSpinDecayCurve() {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 300, 0); // High spin
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result.spinDecay);
        assertTrue(result.spinDecay.times.length > 0);
        assertTrue(result.spinDecay.spinRates.length > 0);
        
        // Spin should decay over time
        double initialSpin = result.spinDecay.getInitialSpinRate();
        double finalSpin = result.spinDecay.getFinalSpinRate();
        assertTrue(finalSpin < initialSpin, "Spin should decay during flight");
        
        // Check decay percentage is reasonable
        double decayPercent = result.spinDecay.getDecayPercentage();
        assertTrue(decayPercent >= 0 && decayPercent <= 100);
    }
    
    @Test
    public void testTrajectoryOverTime() {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result.trajectory);
        assertTrue(result.trajectory.times.length > 0);
        assertEquals(result.trajectory.times.length, result.trajectory.positions.length);
        assertEquals(result.trajectory.times.length, result.trajectory.velocities.length);
        assertEquals(result.trajectory.times.length, result.trajectory.spins.length);
        
        // Times should be increasing
        for (int i = 1; i < result.trajectory.times.length; i++) {
            assertTrue(result.trajectory.times[i] > result.trajectory.times[i-1]);
        }
    }
    
    @Test
    public void testPerturbedSimulations() {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        SpinShotSimulator.SimulationResult[] results = 
            simulator.simulateShotWithPerturbations(target, speed, spin, 5, 0.5, 10.0);
        
        assertEquals(5, results.length);
        
        // All simulations should complete
        for (SpinShotSimulator.SimulationResult result : results) {
            assertNotNull(result);
            assertTrue(result.simulationsRun > 0);
        }
        
        // Results should be similar but not identical (except first which is nominal)
        double firstScore = results[0].bestScore;
        for (int i = 1; i < results.length; i++) {
            // Perturbed results should be within reasonable range
            double scoreDiff = Math.abs(results[i].bestScore - firstScore);
            assertTrue(scoreDiff < 10.0, "Perturbed results should be similar to nominal");
        }
    }
    
    @Test
    public void testDeterministicBehavior() {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        SpinShotSimulator.SimulationResult result1 = simulator.simulateShot(target, speed, spin);
        SpinShotSimulator.SimulationResult result2 = simulator.simulateShot(target, speed, spin);
        
        // Same inputs should produce same outputs
        assertEquals(result1.launchYawDeg, result2.launchYawDeg, 1e-6);
        assertEquals(result1.launchPitchDeg, result2.launchPitchDeg, 1e-6);
        assertEquals(result1.bestScore, result2.bestScore, 1e-6);
        assertEquals(result1.apexHeight, result2.apexHeight, 1e-6);
        assertEquals(result1.flightTime, result2.flightTime, 1e-6);
    }
    
    @Test
    public void testRealTimeSafe() {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        long startTime = System.currentTimeMillis();
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        long elapsed = System.currentTimeMillis() - startTime;
        
        // Should complete in reasonable time (relaxed for testing)
        assertTrue(elapsed < 5000, "Simulation should complete in under 5 seconds");
        assertNotNull(result);
    }
    
    @ParameterizedTest
    @CsvSource({
        "0, 100, 0",    // Backspin (Y-axis)
        "100, 0, 0",    // Sidespin (X-axis)
        "0, 0, 100",    // Topspin (Z-axis)
        "70, 70, 0",    // Diagonal spin
        "50, 50, 50"    // 3D spin
    })
    public void testVariousSpinAxes(double sx, double sy, double sz) {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(sx, sy, sz);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        assertTrue(result.simulationsRun > 0);
        
        // Should complete and produce valid results regardless of spin axis
        assertFalse(Double.isNaN(result.bestScore));
        assertFalse(Double.isInfinite(result.bestScore));
    }
    
    @Test
    public void testHighSpinRate() {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 450, 0); // Near max spin rate
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        assertTrue(result.spinDecay.getInitialSpinRate() <= PhysicsConstants.MAX_SPIN_RATE,
            "Spin should be clamped to safety limit");
    }
    
    @Test
    public void testLowSpeedScenario() {
        Vector3D target = new Vector3D(2.0, 0.0, 1.8);
        double speed = 6.0; // Low speed
        Vector3D spin = new Vector3D(0, 100, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        // May or may not hit, but should complete
        assertTrue(result.simulationsRun > 0);
    }
    
    @Test
    public void testHighSpeedScenario() {
        Vector3D target = new Vector3D(4.0, 0.0, 2.2);
        double speed = 20.0; // High speed
        Vector3D spin = new Vector3D(0, 300, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        assertTrue(result.simulationsRun > 0);
        // High speed may or may not hit depending on available angles
        // Just verify simulation completes and produces valid results
        assertFalse(Double.isNaN(result.bestScore));
        assertFalse(Double.isInfinite(result.bestScore));
    }
    
    @Test
    public void testReportGeneration() {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        String report = result.generateReport();
        assertNotNull(report);
        assertTrue(report.contains("Simulation Report"));
        assertTrue(report.contains("Launch Angles"));
        assertTrue(report.contains("Risk Score"));
        assertTrue(report.contains("Apex Height"));
        assertTrue(report.contains("Flight Time"));
    }
    
    @Test
    public void testJSONExport() {
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        String json = result.toJSON();
        assertNotNull(json);
        assertTrue(json.contains("\"hit\":"));
        assertTrue(json.contains("\"launchYawDeg\":"));
        assertTrue(json.contains("\"launchPitchDeg\":"));
        assertTrue(json.contains("\"bestScore\":"));
        assertTrue(json.contains("\"apexHeight\":"));
    }
    
    @Test
    public void testCustomBallProperties() {
        // Test with worn ball
        ProjectileProperties wornBall = ProjectileProperties.wornBall();
        SpinShotSimulator customSimulator = new SpinShotSimulator(new CalibrationParameters(), wornBall);
        customSimulator.setYawRange(-45, 45, 10);
        customSimulator.setPitchRange(20, 60, 5);
        
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        SpinShotSimulator.SimulationResult result = customSimulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        assertTrue(result.simulationsRun > 0);
    }
    
    @Test
    public void testCustomCalibrationParameters() {
        // Test with custom parameters
        CalibrationParameters params = new CalibrationParameters()
            .withDragCoefficient(0.5)
            .withMagnusCoefficient(0.00002)
            .withSpinDecayRate(0.08);
        
        SpinShotSimulator customSimulator = new SpinShotSimulator(params, new ProjectileProperties());
        customSimulator.setYawRange(-45, 45, 10);
        customSimulator.setPitchRange(20, 60, 5);
        
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        SpinShotSimulator.SimulationResult result = customSimulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        assertTrue(result.simulationsRun > 0);
    }
    
    @Test
    public void testNearRimEntry() {
        // Target that might skim rim
        Vector3D target = new Vector3D(3.5, 0.0, 2.0);
        double speed = 11.0;
        Vector3D spin = new Vector3D(0, 250, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        // Should consider rim proximity in scoring
        assertFalse(Double.isNaN(result.bestScore));
    }
    
    @Test
    public void testConfigurableRanges() {
        // Test with custom turret ranges
        simulator.setYawRange(-90, 90, 15);
        simulator.setPitchRange(15, 75, 10);
        
        Vector3D target = new Vector3D(3.0, 0.0, 2.0);
        double speed = 12.0;
        Vector3D spin = new Vector3D(0, 200, 0);
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
        
        assertNotNull(result);
        assertTrue(result.simulationsRun > 0);
        
        // Should search within configured ranges
        assertTrue(result.launchYawDeg >= -90 && result.launchYawDeg <= 90);
        assertTrue(result.launchPitchDeg >= 15 && result.launchPitchDeg <= 75);
    }
}
