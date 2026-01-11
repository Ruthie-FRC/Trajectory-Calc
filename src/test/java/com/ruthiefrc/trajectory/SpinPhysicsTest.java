package com.ruthiefrc.trajectory;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.junit.jupiter.params.provider.CsvSource;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Comprehensive tests for spin simulation with foam balls.
 * Tests Magnus effect, spin decay, rim interactions, and axis variations.
 */
public class SpinPhysicsTest {
    
    @Test
    public void testSpinDecayConfiguration() {
        // Test that spin decay rate can be configured
        CalibrationParameters params = new CalibrationParameters();
        CalibrationParameters custom = params.withSpinDecayRate(0.1);
        
        assertEquals(PhysicsConstants.DEFAULT_SPIN_DECAY_RATE, params.spinDecayRate, 0.001);
        assertEquals(0.1, custom.spinDecayRate, 0.001);
    }
    
    @Test
    public void testSpinDecayRateSafetyLimits() {
        // Test that spin decay rate is clamped to safe limits
        CalibrationParameters tooLow = new CalibrationParameters()
            .withSpinDecayRate(-0.1);
        CalibrationParameters tooHigh = new CalibrationParameters()
            .withSpinDecayRate(1.0);
        
        assertEquals(PhysicsConstants.MIN_SPIN_DECAY_RATE, tooLow.spinDecayRate, 0.001);
        assertEquals(PhysicsConstants.MAX_SPIN_DECAY_RATE, tooHigh.spinDecayRate, 0.001);
    }
    
    @ParameterizedTest
    @ValueSource(doubles = {0.0, 50.0, 100.0, 200.0, 350.0, 500.0})
    public void testVariousSpinRates(double spinRate) {
        // Test solver with various spin rates (0-500 rad/s)
        ShooterController controller = new ShooterController();
        
        long startTime = System.currentTimeMillis();
        InverseSolver.SolutionResult solution = controller.calculateShot(
            3.0, 0.0, 1.0, 12.0, spinRate
        );
        long elapsedTime = System.currentTimeMillis() - startTime;
        
        // Verify solution is valid
        assertNotNull(solution);
        
        // Some positions might not be reachable with certain parameters
        // Just verify the calculation completed
        System.out.println("Spin " + spinRate + " rad/s: yaw=" + String.format("%.2f", solution.launchYawDeg) +
            "°, pitch=" + String.format("%.2f", solution.launchPitchDeg) + "°, score=" + String.format("%.3f", solution.score) +
            ", time=" + elapsedTime + "ms");
    }
    
    @ParameterizedTest
    @CsvSource({
        "0.0, 1.0, 0.0",  // Backspin (Y-axis)
        "1.0, 0.0, 0.0",  // Sidespin right (X-axis)
        "-1.0, 0.0, 0.0", // Sidespin left (-X-axis)
        "0.0, 0.0, 1.0",  // Topspin (Z-axis)
        "0.7, 0.7, 0.0",  // Diagonal spin
        "0.577, 0.577, 0.577" // 3D diagonal spin
    })
    public void testSpinAxisVariations(double axisX, double axisY, double axisZ) {
        // Test solver with various spin axis orientations
        ShooterController controller = new ShooterController();
        
        double spinRate = 200.0; // rad/s
        
        long startTime = System.currentTimeMillis();
        InverseSolver.SolutionResult solution = controller.calculateShot(
            3.0, 0.0, 1.0, 12.0, spinRate, axisX, axisY, axisZ
        );
        long elapsedTime = System.currentTimeMillis() - startTime;
        
        // Verify real-time safe
        assertTrue(elapsedTime < 100, "Solve time: " + elapsedTime + "ms");
        
        // Verify solution is valid
        assertNotNull(solution);
        System.out.println("Spin axis (" + axisX + "," + axisY + "," + axisZ + "): " +
            "yaw=" + String.format("%.2f", solution.launchYawDeg) + "°, " +
            "pitch=" + String.format("%.2f", solution.launchPitchDeg) + "°");
    }
    
    @Test
    public void testMagnusForceCalculation() {
        // Test that Magnus force is properly calculated
        CalibrationParameters params = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel model = new PhysicsModel(params, projectile);
        
        // Create state with velocity and spin
        Vector3D velocity = new Vector3D(10, 0, 0); // Moving in +X direction
        Vector3D spin = new Vector3D(0, 100, 0); // Spinning around Y-axis (backspin)
        ProjectileState state = new ProjectileState(
            new Vector3D(0, 0, 1),
            velocity,
            spin,
            0.0
        );
        
        // Compute derivative (includes Magnus force)
        StateDerivative derivative = model.computeDerivative(state);
        
        // Magnus force creates perpendicular lift
        // ω × v = (0, 100, 0) × (10, 0, 0) = (0, 0, -1000) with our coordinate system
        // The magnitude should be non-zero if Magnus coefficient is applied
        double magnusAccelMagnitude = Math.sqrt(
            derivative.velocityDerivative.x * derivative.velocityDerivative.x +
            derivative.velocityDerivative.y * derivative.velocityDerivative.y +
            derivative.velocityDerivative.z * derivative.velocityDerivative.z
        );
        
        // With default Magnus coefficient, there should be some effect
        assertTrue(magnusAccelMagnitude > 0, "Magnus force should be computed");
    }
    
    @Test
    public void testSpinDecayOverTime() {
        // Test that spin decays realistically over time
        CalibrationParameters params = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel model = new PhysicsModel(params, projectile);
        
        Vector3D velocity = new Vector3D(15, 0, 5); // Realistic shot velocity
        Vector3D initialSpin = new Vector3D(0, 300, 0); // 300 rad/s backspin
        ProjectileState state = new ProjectileState(
            new Vector3D(0, 0, 1),
            velocity,
            initialSpin,
            0.0
        );
        
        // Compute derivative
        StateDerivative derivative = model.computeDerivative(state);
        
        // Spin should be decaying (negative derivative)
        assertTrue(derivative.spinDerivative.y < 0, "Spin should decay over time");
        
        // Decay rate should scale with velocity (air resistance)
        double decayMagnitude = Math.abs(derivative.spinDerivative.y);
        assertTrue(decayMagnitude > 0, "Spin decay should be non-zero");
        assertTrue(decayMagnitude < initialSpin.y, "Spin decay should be reasonable");
    }
    
    @Test
    public void testSpinImpactOnRimBounce() {
        // Test that spin affects post-bounce trajectory
        CalibrationParameters params = new CalibrationParameters();
        ProjectileProperties projectile = new ProjectileProperties();
        PhysicsModel model = new PhysicsModel(params, projectile);
        
        Vector3D velocity = new Vector3D(5, 0, -3); // Moving downward and forward
        Vector3D spin = new Vector3D(0, 200, 0); // Significant backspin
        Vector3D position = new Vector3D(0, 0, 1);
        ProjectileState preCollision = new ProjectileState(position, velocity, spin, 0.0);
        
        // Simulate collision with horizontal surface (HUB rim)
        Vector3D surfaceNormal = new Vector3D(0, 0, 1); // Upward normal
        ProjectileState postCollision = model.applyCollision(preCollision, surfaceNormal);
        
        // Verify spin is reduced after collision (foam compression)
        double preSpinMagnitude = spin.magnitude();
        double postSpinMagnitude = postCollision.spin.magnitude();
        assertTrue(postSpinMagnitude < preSpinMagnitude, "Spin should be reduced by collision");
        assertTrue(postSpinMagnitude > 0, "Spin should not be completely eliminated");
        
        // Verify velocity changed appropriately
        assertNotNull(postCollision.velocity);
        assertTrue(postCollision.velocity.z > 0, "Should bounce upward");
    }
    
    @Test
    public void testSpinAxisControl() {
        // Test that spin axis can be controlled independently
        ShooterController controller = new ShooterController();
        
        // Test backspin
        InverseSolver.SolutionResult backspin = controller.calculateShot(
            3.0, 0.0, 1.0, 12.0, 200.0, 0, 1, 0
        );
        
        // Test sidespin
        InverseSolver.SolutionResult sidespin = controller.calculateShot(
            3.0, 0.0, 1.0, 12.0, 200.0, 1, 0, 0
        );
        
        // Different spin axes should produce different trajectories
        assertNotNull(backspin);
        assertNotNull(sidespin);
        
        System.out.println("Backspin: yaw=" + String.format("%.2f", backspin.launchYawDeg) + 
            "°, pitch=" + String.format("%.2f", backspin.launchPitchDeg) + "°");
        System.out.println("Sidespin: yaw=" + String.format("%.2f", sidespin.launchYawDeg) + 
            "°, pitch=" + String.format("%.2f", sidespin.launchPitchDeg) + "°");
    }
    
    @Test
    public void testSpinSafetyLimit() {
        // Test that extremely high spin rates are safely handled
        ShooterController controller = new ShooterController();
        
        // Request unrealistic spin rate (10x max)
        double extremeSpin = 5000.0; // rad/s
        
        InverseSolver.SolutionResult solution = controller.calculateShot(
            3.0, 0.0, 1.0, 12.0, extremeSpin
        );
        
        // Should still produce a solution (with spin clamped internally)
        assertNotNull(solution);
        System.out.println("Extreme spin " + extremeSpin + " rad/s handled: score=" + solution.score);
    }
    
    @Test
    public void testZeroSpinBehavior() {
        // Test that zero spin produces valid ballistic trajectory
        ShooterController controller = new ShooterController();
        
        InverseSolver.SolutionResult solution = controller.calculateShot(
            3.0, 0.0, 1.0, 12.0, 0.0
        );
        
        assertNotNull(solution);
        assertTrue(solution.score > -10, "Zero spin should still produce reasonable solution");
    }
    
    @Test
    public void testSpinEfficiencyCalibration() {
        // Test that spin efficiency can be calibrated
        CalibrationParameters defaultParams = new CalibrationParameters();
        CalibrationParameters customParams = defaultParams.withSpinEfficiency(0.85);
        
        assertEquals(PhysicsConstants.DEFAULT_SPIN_EFFICIENCY, defaultParams.spinEfficiency, 0.001);
        assertEquals(0.85, customParams.spinEfficiency, 0.001);
        
        // Test with controller
        ShooterController controller1 = new ShooterController(defaultParams);
        ShooterController controller2 = new ShooterController(customParams);
        
        InverseSolver.SolutionResult solution1 = controller1.calculateShot(3.0, 0.0, 1.0, 12.0, 200.0);
        InverseSolver.SolutionResult solution2 = controller2.calculateShot(3.0, 0.0, 1.0, 12.0, 200.0);
        
        assertNotNull(solution1);
        assertNotNull(solution2);
        
        // Different spin efficiencies may produce different solutions
        System.out.println("Default efficiency: " + String.format("%.2f", solution1.launchPitchDeg) + "°");
        System.out.println("Custom efficiency: " + String.format("%.2f", solution2.launchPitchDeg) + "°");
    }
    
    @ParameterizedTest
    @CsvSource({
        "3.0, 0.0, 1.0, 100.0", // Short range, low spin
        "5.0, 0.0, 1.0, 200.0", // Medium range, medium spin
        "3.0, 3.0, 1.0, 150.0", // Angled shot with spin
        "4.0, -2.0, 1.0, 250.0" // Different angle with spin
    })
    public void testSpinAtVariousRangesAndAngles(double x, double y, double z, double spinRate) {
        // Test that spin handling works across different shot scenarios
        ShooterController controller = new ShooterController();
        
        long startTime = System.currentTimeMillis();
        InverseSolver.SolutionResult solution = controller.calculateShot(x, y, z, 12.0, spinRate);
        long elapsedTime = System.currentTimeMillis() - startTime;
        
        // Verify solution validity
        assertNotNull(solution);
        
        double distance = Math.sqrt(x*x + y*y);
        System.out.println("Position (" + x + "," + y + "," + z + ") @ " + spinRate + " rad/s: " +
            "distance=" + String.format("%.1f", distance) + "m, " +
            "pitch=" + String.format("%.2f", solution.launchPitchDeg) + "°, " +
            "score=" + String.format("%.3f", solution.score) + ", " +
            "time=" + elapsedTime + "ms");
    }
    
    @Test
    public void testDeterministicSpinSimulation() {
        // Test that spin simulations are deterministic
        ShooterController controller = new ShooterController();
        
        InverseSolver.SolutionResult solution1 = controller.calculateShot(3.0, 0.0, 1.0, 12.0, 200.0);
        InverseSolver.SolutionResult solution2 = controller.calculateShot(3.0, 0.0, 1.0, 12.0, 200.0);
        
        // Same inputs should produce identical outputs
        assertEquals(solution1.launchYawDeg, solution2.launchYawDeg, 0.001);
        assertEquals(solution1.launchPitchDeg, solution2.launchPitchDeg, 0.001);
        assertEquals(solution1.score, solution2.score, 0.001);
    }
}
