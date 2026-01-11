# ShooterAimingController User Guide

## Overview

`ShooterAimingController` is a high-level control class that automatically computes optimal turret yaw and hood pitch angles for FRC shooter systems. Given the robot's field pose and target location, it handles all coordinate transformations, trajectory evaluation, and safety checks to provide smooth, reliable aiming commands.

## Key Features

- **Field-relative to robot-relative coordinate transformation**: Automatically handles robot heading and position
- **Comprehensive trajectory evaluation**: Uses SpinShotSimulator with full physics simulation
- **Multi-factor risk scoring**: Considers bounce-out risk, rim proximity, and entry quality
- **Mechanical limit enforcement**: Respects turret and hood constraints
- **Smooth command output**: Configurable smoothing to prevent oscillation
- **Solution caching**: Faster convergence for repeated field positions
- **Safety validation**: Returns "no-shot" state when no viable solution exists
- **Confidence metrics**: Perturbation testing provides reliability estimates

## Quick Start

### Basic Setup

```java
import com.ruthiefrc.trajectory.*;

// Create controller with default parameters
ShooterAimingController controller = new ShooterAimingController();

// Configure mechanical limits
controller.setTurretYawLimits(
    Math.toRadians(-90),  // Min yaw: -90°
    Math.toRadians(90)    // Max yaw: +90°
);

controller.setHoodPitchLimits(
    Math.toRadians(15),   // Min pitch: 15°
    Math.toRadians(75)    // Max pitch: 75°
);

// Configure search resolution (degrees)
controller.setSearchResolution(
    5.0,  // Turret yaw step: 5°
    2.0   // Hood pitch step: 2°
);
```

### Continuous Aiming Loop

```java
// In your robot's periodic() or execute() method
public void periodic() {
    // Get current robot pose (from odometry, vision, etc.)
    double robotX = odometry.getX();
    double robotY = odometry.getY();
    double robotHeading = odometry.getHeading();  // Radians
    
    // Create robot pose
    ShooterAimingController.RobotPose robotPose = 
        new ShooterAimingController.RobotPose(robotX, robotY, robotHeading);
    
    // Target position (field coordinates)
    Vector3D target = new Vector3D(
        HUB_X,  // Target X position
        HUB_Y,  // Target Y position
        HUB_Z   // Target height
    );
    
    // Update aiming solution
    ShooterAimingController.AimingSolution solution = 
        controller.update(robotPose, target);
    
    // Apply commands if shot is available
    if (solution.hasShot) {
        turretSubsystem.setYaw(solution.turretYawRad);
        hoodSubsystem.setPitch(solution.hoodPitchRad);
        shooterSubsystem.setSpeed(solution.shooterSpeedMps);
        
        // Log confidence and risk metrics
        System.out.println("Confidence: " + solution.confidence);
        System.out.println("Risk: " + solution.riskScore);
    } else {
        // No viable shot - log reason
        System.out.println("No shot: " + solution.errorMessage);
    }
}
```

## Configuration Options

### Mechanical Limits

```java
// Turret yaw limits (radians, robot-relative)
controller.setTurretYawLimits(minYawRad, maxYawRad);

// Hood pitch limits (radians from horizontal)
controller.setHoodPitchLimits(minPitchRad, maxPitchRad);
```

### Search Resolution

Finer resolution = more accurate but slower. Coarser = faster but less precise.

```java
// Trade-off: accuracy vs speed
controller.setSearchResolution(
    5.0,  // Turret: 5° steps (good balance)
    2.0   // Hood: 2° steps (good balance)
);

// High accuracy (slower)
controller.setSearchResolution(2.0, 1.0);

// High speed (less accurate)
controller.setSearchResolution(10.0, 5.0);
```

### Spin Configuration

```java
// Set default spin rate and axis
controller.setDefaultSpin(
    200.0,  // Spin rate (rad/s)
    0,      // Spin axis X
    1,      // Spin axis Y (backspin)
    0       // Spin axis Z
);

// High backspin
controller.setDefaultSpin(300.0, 0, 1, 0);

// Sidespin
controller.setDefaultSpin(150.0, 1, 0, 0);
```

### Shooter Parameters

```java
// Default shooter speed
controller.setDefaultShooterSpeed(12.0);  // m/s

// Shooter height above robot origin
controller.setShooterHeight(0.5);  // meters
```

### Risk and Safety

```java
// Maximum acceptable risk score (0-1)
controller.setMaxAcceptableRisk(0.5);  // Default: 0.5

// Stricter (fewer shots accepted)
controller.setMaxAcceptableRisk(0.3);

// More lenient (more shots accepted)
controller.setMaxAcceptableRisk(0.7);
```

### Smoothing

Prevents command oscillation when robot or target moves slightly.

```java
// Smoothing factor (0 = no smoothing, 1 = full smoothing)
controller.setSmoothingFactor(0.3);  // Default: 0.3

// No smoothing (instant response)
controller.setSmoothingFactor(0.0);

// Heavy smoothing (slow response)
controller.setSmoothingFactor(0.7);
```

### Confidence Testing

```java
// Enable/disable perturbation testing for confidence
controller.enablePerturbationTesting(true);  // Default: true

// Disable for faster computation (no confidence metric)
controller.enablePerturbationTesting(false);
```

## AdvantageKit Integration

```java
// Update with optional logging
ShooterAimingController.AimingSolution solution = 
    controller.update(robotPose, target, "Shooter/Aiming/");

// Logs appear in AdvantageScope:
// - Shooter/Aiming/Inputs/RobotX, RobotY, RobotHeading
// - Shooter/Aiming/Inputs/TargetX, TargetY, TargetZ
// - Shooter/Aiming/Outputs/HasShot
// - Shooter/Aiming/Outputs/TurretYawDeg
// - Shooter/Aiming/Outputs/HoodPitchDeg
// - Shooter/Aiming/Outputs/ShooterSpeed
// - Shooter/Aiming/Metrics/Confidence
// - Shooter/Aiming/Metrics/RiskScore
```

## Solution Object

```java
public class AimingSolution {
    boolean hasShot;              // True if valid shot exists
    double turretYawRad;          // Robot-relative turret yaw (radians)
    double hoodPitchRad;          // Hood pitch from horizontal (radians)
    double shooterSpeedMps;       // Required shooter speed (m/s)
    double spinRate;              // Spin rate (rad/s)
    double spinAxisX, Y, Z;       // Spin axis components
    double confidence;            // Confidence metric (0-1)
    double riskScore;             // Risk score (0-1, lower is better)
    String errorMessage;          // Error message if no shot
    
    // Convenience methods
    double getTurretYawDeg();     // Get turret yaw in degrees
    double getHoodPitchDeg();     // Get hood pitch in degrees
}
```

## Advanced Usage

### Cache Management

```java
// Clear cached solutions (e.g., after field state change)
controller.clearCache();
```

### Custom Calibration

```java
// Create controller with custom physics parameters
CalibrationParameters calibration = new CalibrationParameters()
    .withDragCoefficient(0.5)
    .withMagnusCoefficient(0.000015)
    .withSpinEfficiency(0.93);

ProjectileProperties projectile = new ProjectileProperties(
    0.15,   // Diameter (m)
    0.215   // Mass (kg)
);

ShooterAimingController controller = 
    new ShooterAimingController(calibration, projectile);
```

### Multiple Targets

```java
// Switch between multiple targets dynamically
Vector3D hubTarget = new Vector3D(8.23, 4.11, 2.64);
Vector3D upperTarget = new Vector3D(8.23, 4.11, 3.5);

if (targetingUpper) {
    solution = controller.update(robotPose, upperTarget);
} else {
    solution = controller.update(robotPose, hubTarget);
}
```

## Performance Considerations

### Computation Time

- Typical: 100-500ms per update (depends on search resolution)
- With caching: 50-200ms for repeated positions
- Real-time safe for robot control loops at 20-50 Hz

### Optimization Tips

1. **Coarser search resolution** for real-time operation:
   ```java
   controller.setSearchResolution(10.0, 5.0);
   ```

2. **Disable perturbation testing** during matches:
   ```java
   controller.enablePerturbationTesting(false);
   ```

3. **Use caching** - don't clear cache unnecessarily

4. **Call update() at moderate rate** (20-50 Hz, not 200 Hz)

## Coordinate Systems

### Field Coordinates
- Origin: Typically field corner or center
- X-axis: Along field length
- Y-axis: Along field width
- Heading: 0 = +X axis, increases counterclockwise

### Robot Coordinates
- Origin: Robot center
- X-axis: Forward
- Y-axis: Left
- Turret yaw: 0 = forward, positive = left

### Transformation
The controller automatically transforms field-relative targets to robot-relative coordinates:
```
dx = targetX - robotX
dy = targetY - robotY
xRobot = dx * cos(-heading) - dy * sin(-heading)
yRobot = dx * sin(-heading) + dy * cos(-heading)
```

## Troubleshooting

### "No shot" errors

**Common causes:**
1. Target out of range (too far or too high)
2. Trajectory has high bounce-out risk
3. No solution within mechanical limits
4. Risk score exceeds threshold

**Solutions:**
- Increase `maxAcceptableRisk`
- Check target position is correct
- Verify mechanical limits are reasonable
- Increase shooter speed

### Oscillating commands

**Cause:** Rapid updates with no smoothing

**Solution:**
```java
controller.setSmoothingFactor(0.3);  // Add smoothing
```

### Slow performance

**Causes:**
- Fine search resolution
- Perturbation testing enabled
- High update rate

**Solutions:**
```java
controller.setSearchResolution(10.0, 5.0);  // Coarser
controller.enablePerturbationTesting(false);  // Disable
```

### Inaccurate solutions

**Causes:**
- Coarse search resolution
- Wrong calibration parameters
- Incorrect shooter height

**Solutions:**
```java
controller.setSearchResolution(2.0, 1.0);  // Finer
controller.setShooterHeight(correctHeight);
// Use calibrated physics parameters
```

## Best Practices

1. **Configure once in robot init**, update continuously in periodic
2. **Use moderate search resolution** (5° yaw, 2° pitch) for balance
3. **Enable smoothing** (0.2-0.4) to prevent oscillation
4. **Check `hasShot`** before applying commands
5. **Log confidence and risk** for tuning and debugging
6. **Don't clear cache** unless field state changes significantly
7. **Use AdvantageKit logging** during practice for analysis
8. **Disable perturbation testing** during competitions for speed

## Integration Example

Complete subsystem example:

```java
public class TurretSubsystem extends SubsystemBase {
    private final ShooterAimingController aimingController;
    private final Odometry odometry;
    
    public TurretSubsystem(Odometry odometry) {
        this.odometry = odometry;
        this.aimingController = new ShooterAimingController();
        
        // Configure
        aimingController.setTurretYawLimits(Math.toRadians(-90), Math.toRadians(90));
        aimingController.setHoodPitchLimits(Math.toRadians(15), Math.toRadians(75));
        aimingController.setSearchResolution(5.0, 2.0);
        aimingController.setDefaultSpin(200.0, 0, 1, 0);
        aimingController.setSmoothingFactor(0.3);
    }
    
    @Override
    public void periodic() {
        // Get robot pose
        ShooterAimingController.RobotPose pose = new ShooterAimingController.RobotPose(
            odometry.getX(),
            odometry.getY(),
            odometry.getHeading()
        );
        
        // Target (HUB center)
        Vector3D target = new Vector3D(8.23, 4.11, 2.64);
        
        // Update aiming
        ShooterAimingController.AimingSolution solution = 
            aimingController.update(pose, target, "Shooter/Aiming/");
        
        // Apply or idle
        if (solution.hasShot && wantToShoot) {
            turretMotor.set(pidController.calculate(
                turretEncoder.getPosition(),
                solution.turretYawRad
            ));
            hoodMotor.set(pidController.calculate(
                hoodEncoder.getPosition(),
                solution.hoodPitchRad
            ));
            shooterMotor.setVelocity(solution.shooterSpeedMps);
        } else {
            // Idle or default position
            turretMotor.set(0);
        }
    }
    
    public boolean isReadyToShoot() {
        // Check if turret/hood are at setpoint
        return turretAtSetpoint() && hoodAtSetpoint() && shooterAtSpeed();
    }
}
```

## See Also

- `SPIN_SHOT_SIMULATOR.md` - SpinShotSimulator documentation
- `SPIN_PHYSICS.md` - Spin physics modeling
- `ADVANTAGEKIT_TUNING.md` - Calibration workflows
- `QUICKSTART.md` - Library integration guide
