package com.ruthiefrc.trajectory;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for ShooterAimingController.
 */
class ShooterAimingControllerTest {
    
    private static final double TOLERANCE = 0.01;
    
    /**
     * Test basic aiming to a target in front of robot.
     */
    @Test
    void testBasicAiming() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);  // Coarse search for speed
        controller.enablePerturbationTesting(false);  // Disable for speed
        
        // Robot at origin, facing +X
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        
        // Target 3m in front, 2.5m high
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        assertTrue(solution.hasShot);
        assertTrue(solution.confidence > 0);
        assertTrue(solution.riskScore < 1.0);
        
        // Turret should be roughly forward (small yaw)
        assertTrue(Math.abs(solution.turretYawRad) < Math.toRadians(10));
        
        // Hood should be elevated (positive pitch)
        assertTrue(solution.hoodPitchRad > 0);
        assertTrue(solution.hoodPitchRad < Math.PI / 2);
    }
    
    /**
     * Test field-to-robot coordinate transformation.
     */
    @Test
    void testCoordinateTransformation() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        
        // Robot at (2, 3), facing +Y (90 degrees)
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(2.0, 3.0, Math.PI / 2);
        
        // Target at (2, 6, 2.5) - directly in front of robot in field coords
        Vector3D target = new Vector3D(2.0, 6.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        assertTrue(solution.hasShot);
        
        // Turret should be roughly forward (robot is facing target)
        assertTrue(Math.abs(solution.turretYawRad) < Math.toRadians(15));
    }
    
    /**
     * Test turret aiming to the side.
     */
    @Test
    void testSideTarget() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        
        // Robot at origin, facing +X
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        
        // Target to the right side (3m right, 2.5m high)
        Vector3D target = new Vector3D(0.0, -3.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        assertTrue(solution.hasShot);
        
        // Turret should be turned significantly (target is to the side)
        assertTrue(Math.abs(solution.turretYawRad) > Math.toRadians(60));
    }
    
    /**
     * Test rejection of unreachable targets (too far).
     */
    @Test
    void testUnreachableTarget() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(15.0, 10.0);  // Coarse for speed
        controller.enablePerturbationTesting(false);
        controller.setMaxAcceptableRisk(0.3);  // Strict
        
        // Robot at origin
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        
        // Target very far away
        Vector3D target = new Vector3D(20.0, 0.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        assertFalse(solution.hasShot);
        assertFalse(solution.errorMessage.isEmpty());
    }
    
    /**
     * Test mechanical limits enforcement.
     */
    @Test
    void testMechanicalLimits() {
        ShooterAimingController controller = new ShooterAimingController();
        
        // Set tight turret limits
        controller.setTurretYawLimits(Math.toRadians(-30), Math.toRadians(30));
        controller.setHoodPitchLimits(Math.toRadians(20), Math.toRadians(60));
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        
        // Robot at origin, facing +X
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        
        // Target forward
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        if (solution.hasShot) {
            // Solution should respect limits
            assertTrue(solution.turretYawRad >= Math.toRadians(-30) - TOLERANCE);
            assertTrue(solution.turretYawRad <= Math.toRadians(30) + TOLERANCE);
            assertTrue(solution.hoodPitchRad >= Math.toRadians(20) - TOLERANCE);
            assertTrue(solution.hoodPitchRad <= Math.toRadians(60) + TOLERANCE);
        }
    }
    
    /**
     * Test confidence calculation with perturbation testing.
     */
    @Test
    void testConfidenceCalculation() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(true);
        
        // Robot at origin
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        
        // Easy target (close, good angle)
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        if (solution.hasShot) {
            assertTrue(solution.confidence >= 0.0 && solution.confidence <= 1.0);
            // For an easy target, confidence should be reasonably high
            assertTrue(solution.confidence > 0.3);
        }
    }
    
    /**
     * Test smoothing to prevent oscillation.
     */
    @Test
    void testSmoothing() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        controller.setSmoothingFactor(0.5);  // Moderate smoothing
        
        // Robot at origin
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        
        // First target
        Vector3D target1 = new Vector3D(3.0, 0.0, 2.5);
        ShooterAimingController.AimingSolution solution1 = 
            controller.update(robotPose, target1);
        
        // Slightly different target
        Vector3D target2 = new Vector3D(3.1, 0.1, 2.5);
        ShooterAimingController.AimingSolution solution2 = 
            controller.update(robotPose, target2);
        
        if (solution1.hasShot && solution2.hasShot) {
            // Second solution should be smoothed (not jump to exact new value)
            double yawChange = Math.abs(solution2.turretYawRad - solution1.turretYawRad);
            assertTrue(yawChange < Math.toRadians(10));
        }
    }
    
    /**
     * Test cache clearing.
     */
    @Test
    void testCacheClear() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        
        // Robot at origin
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);
        
        // Get initial solution
        ShooterAimingController.AimingSolution solution1 = 
            controller.update(robotPose, target);
        
        // Clear cache
        controller.clearCache();
        
        // Get new solution (should not crash)
        ShooterAimingController.AimingSolution solution2 = 
            controller.update(robotPose, target);
        
        // Both should be valid or both invalid (deterministic)
        assertEquals(solution1.hasShot, solution2.hasShot);
    }
    
    /**
     * Test custom spin configuration.
     */
    @Test
    void testCustomSpin() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        
        // Set high backspin
        controller.setDefaultSpin(300.0, 0, 1, 0);
        
        // Robot at origin
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        if (solution.hasShot) {
            assertEquals(300.0, solution.spinRate, TOLERANCE);
            assertEquals(1.0, solution.spinAxisY, TOLERANCE);
        }
    }
    
    /**
     * Test deterministic behavior for same inputs.
     */
    @Test
    void testDeterministicBehavior() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        controller.setSmoothingFactor(0.0);  // Disable smoothing
        controller.clearCache();  // Clear any cached state
        
        // Robot at origin
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);
        
        // Get first solution
        ShooterAimingController.AimingSolution solution1 = 
            controller.update(robotPose, target);
        
        // Create new controller with same settings
        ShooterAimingController controller2 = new ShooterAimingController();
        controller2.setSearchResolution(10.0, 5.0);
        controller2.enablePerturbationTesting(false);
        controller2.setSmoothingFactor(0.0);
        
        // Get second solution
        ShooterAimingController.AimingSolution solution2 = 
            controller2.update(robotPose, target);
        
        // Solutions should be identical
        assertEquals(solution1.hasShot, solution2.hasShot);
        
        if (solution1.hasShot && solution2.hasShot) {
            assertEquals(solution1.turretYawRad, solution2.turretYawRad, Math.toRadians(0.5));
            assertEquals(solution1.hoodPitchRad, solution2.hoodPitchRad, Math.toRadians(0.5));
        }
    }
    
    /**
     * Test multiple poses and targets.
     */
    @Test
    void testMultiplePositions() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(15.0, 10.0);  // Coarse for speed
        controller.enablePerturbationTesting(false);
        
        // Test array of robot poses and targets
        ShooterAimingController.RobotPose[] poses = {
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0),
            new ShooterAimingController.RobotPose(2.0, 1.0, Math.PI / 4),
            new ShooterAimingController.RobotPose(4.0, 2.0, Math.PI / 2),
            new ShooterAimingController.RobotPose(1.0, -1.0, -Math.PI / 4)
        };
        
        Vector3D[] targets = {
            new Vector3D(3.0, 0.0, 2.5),
            new Vector3D(5.0, 4.0, 2.5),
            new Vector3D(4.0, 6.0, 2.5),
            new Vector3D(4.0, 1.0, 2.5)
        };
        
        for (int i = 0; i < poses.length; i++) {
            ShooterAimingController.AimingSolution solution = 
                controller.update(poses[i], targets[i]);
            
            // Each should either succeed or fail gracefully
            if (solution.hasShot) {
                assertTrue(Math.abs(solution.turretYawRad) <= Math.PI);
                assertTrue(solution.hoodPitchRad > 0 && solution.hoodPitchRad < Math.PI / 2);
            } else {
                assertFalse(solution.errorMessage.isEmpty());
            }
        }
    }
    
    /**
     * Test that bounce-out prone trajectories are rejected.
     */
    @Test
    void testBounceOutRejection() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        controller.setMaxAcceptableRisk(0.2);  // Very strict
        
        // Robot at origin
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        
        // Target that might have high bounce-out risk
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        if (solution.hasShot) {
            // If a shot is found, risk should be below threshold
            assertTrue(solution.riskScore <= 0.2 + TOLERANCE);
        }
    }
    
    /**
     * Test toString method.
     */
    @Test
    void testToString() {
        ShooterAimingController controller = new ShooterAimingController();
        controller.setSearchResolution(10.0, 5.0);
        controller.enablePerturbationTesting(false);
        
        // Robot at origin
        ShooterAimingController.RobotPose robotPose = 
            new ShooterAimingController.RobotPose(0.0, 0.0, 0.0);
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);
        
        ShooterAimingController.AimingSolution solution = 
            controller.update(robotPose, target);
        
        String str = solution.toString();
        assertNotNull(str);
        assertFalse(str.isEmpty());
        
        if (solution.hasShot) {
            assertTrue(str.toLowerCase().contains("turret"));
        } else {
            assertTrue(str.contains("NO SHOT"));
        }
    }
    
    /**
     * Test helper methods for degree conversion.
     */
    @Test
    void testDegreeConversion() {
        ShooterAimingController.AimingSolution solution = 
            new ShooterAimingController.AimingSolution(
                true, Math.PI / 4, Math.PI / 3, 12.0, 200.0, 0, 1, 0, 0.9, 0.1, ""
            );
        
        assertEquals(45.0, solution.getTurretYawDeg(), TOLERANCE);
        assertEquals(60.0, solution.getHoodPitchDeg(), TOLERANCE);
    }
}
