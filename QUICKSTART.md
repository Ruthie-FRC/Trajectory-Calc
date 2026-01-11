# Quick Start Guide

This guide covers installation and basic usage. Most teams can integrate the library in under 10 minutes.

## Installation

### Add Vendordep

1. Download `vendordeps/TrajectoryCalc.json` from this repository
2. Open your robot project in VS Code
3. Press `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (Mac)
4. Type "WPILib: Manage Vendor Libraries"
5. Select "Install new library (offline)"
6. Navigate to the downloaded JSON file
7. Select it

The library will be added to your project dependencies.

## Basic Integration

### Shooter Subsystem

Create or update your shooter subsystem:

```java
package frc.robot.subsystems;

import com.ruthiefrc.trajectory.*;
import com.ruthiefrc.trajectory.advantagekit.AdvantageKitTrajectoryHelper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final AdvantageKitTrajectoryHelper trajectory;
    
    // Robot constants
    private static final double SHOOTER_HEIGHT = 0.8;  // meters
    private static final double LAUNCH_SPEED = 12.0;   // m/s
    private static final double SPIN_RATE = 100.0;     // rad/s
    
    private InverseSolver.SolutionResult lastSolution;
    
    public ShooterSubsystem() {
        // Basic setup with defaults
        trajectory = new AdvantageKitTrajectoryHelper("Shooter/Trajectory");
        
        // Alternative: Initialize with custom calibration
        // CalibrationParameters params = new CalibrationParameters()
        //     .withSpeedEfficiency(0.92)
        //     .withDragCoefficient(0.45);
        // trajectory = new AdvantageKitTrajectoryHelper("Shooter/Trajectory", params);
    }
    
    public void calculateAim(Pose2d robotPose) {
        // Computes launch angles and logs to AdvantageKit
        lastSolution = trajectory.calculateAndLog(
            robotPose.getX(), 
            robotPose.getY(),
            SHOOTER_HEIGHT,
            LAUNCH_SPEED,
            SPIN_RATE
        );
    }
    
    public void aimShooter() {
        if (lastSolution != null && lastSolution.isHit()) {
            // Apply angles to your hardware
            // setTurretAngle(lastSolution.launchYawDeg);
            // setHoodAngle(lastSolution.launchPitchDeg);
            
            System.out.println("Aim: Yaw=" + lastSolution.launchYawDeg + 
                             "° Pitch=" + lastSolution.launchPitchDeg + "°");
        }
    }
    
    public void logShotResult(Pose2d robotPose, boolean hit) {
        trajectory.logShotResult(
            robotPose.getX(), robotPose.getY(),
            lastSolution, LAUNCH_SPEED, SPIN_RATE, hit
        );
    }
    
    // Runtime Configuration
    
    /**
     * Configure for worn or compressed balls.
     */
    public void useWornBall() {
        trajectory.getController().updateProjectileProperties(
            ProjectileProperties.wornBall()
        );
    }
    
    /**
     * Reduce CPU load during competition.
     */
    public void disableLogging() {
        trajectory.getController().setLoggingEnabled(false);
    }
    
    /**
     * Enable detailed logging for practice.
     */
    public void enableLogging() {
        trajectory.getController().setLoggingEnabled(true);
    }
}
```

## Command Integration

Bind aiming to a command:
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAimAndShoot extends Command {
    private final ShooterSubsystem shooter;
    private final DriveSubsystem drive;
    
    public AutoAimAndShoot(ShooterSubsystem shooter, DriveSubsystem drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        // Calculate aim when command starts
        shooter.calculateAim(drive.getPose());
    }
    
    @Override
    public void execute() {
        // Keep aiming
        shooter.aimShooter();
    }
    
    @Override
    public void end(boolean interrupted) {
        // Log result (you need to detect if shot scored)
        boolean hit = false; // TODO: Implement hit detection
        shooter.logShotResult(drive.getPose(), hit);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Hold button to aim
    }
}
```

Bind to a button in RobotContainer:

```java
driverController.y().whileTrue(new AutoAimAndShoot(shooter, drive));
```

## Viewing Logs

After running your robot:

1. Enable the robot and drive around
2. Press the aiming button
3. Open AdvantageScope
4. Load the log file
5. Navigate to `Shooter/Trajectory/`

Log structure:
- `Input/*` - Robot position and commanded speeds
- `Output/*` - Calculated angles and predictions
- `Calibration/*` - Current tuning parameters

## Parameter Tuning

After collecting practice data, adjust parameters based on performance:

### Manual Adjustment

If shots consistently fall short:
```java
// Adjust drag or speed efficiency
public void tuneDrag() {
    CalibrationParameters newParams = trajectory.getCalibrationParameters()
        .withDragCoefficient(0.45)        // Lower if shots are short
        .withSpeedEfficiency(0.93);        // Raise if speed is low
    
    trajectory.updateCalibration(newParams);
}
```

### Automatic Calibration

After collecting 10+ logged shots with results:

```java
// Run calibration after practice
public void calibrate() {
    boolean updated = trajectory.calibrate();
    if (updated) {
        System.out.println("Calibration updated");
        System.out.println(trajectory.getCalibrationStats());
    }
}
```

Call this method in test mode after practice sessions.

## Advanced Configuration

### Incremental Calibration

Enable continuous parameter adjustment during practice:

```java
// Enable incremental calibration for practice
controller.setIncrementalCalibrationEnabled(true);
controller.setLearningRate(0.05); // 5% adjustment per shot

// Parameters automatically update after each logged shot
controller.logShot(x, y, z, solution, speed, spin, hit);

// Disable during competition for stability
controller.setIncrementalCalibrationEnabled(false);
```

**Behavior:**
- Analyzes recent performance after each logged shot
- Adjusts drag and speed efficiency when hit rate drops below 60%
- Learning rate controls adaptation speed (0.0-1.0, default: 0.05)
- Requires 5 baseline shots before updates begin
- Changes apply automatically without manual calibration

### Pre-Seeded Optimization

The solver caches previous successful shots for faster convergence:

```java
// First shot uses geometric estimate
var solution1 = controller.calculateShot(3.0, 0.0, 0.5, 12.0, 100.0);

// Subsequent shots from nearby positions use cached angles
var solution2 = controller.calculateShot(3.1, 0.1, 0.5, 12.0, 100.0);

// Pre-seeding is enabled by default
// Access through solver for custom control if needed
```

**Performance:**
- Faster convergence for repeated shots
- More consistent solutions
- Reduces computation time by 10-30%
- Automatic caching on 0.5m grid

### Ball Properties

Configure for different ball conditions:

```java
// Worn or compressed balls
controller.updateProjectileProperties(ProjectileProperties.wornBall());

// Heavy balls (0.500 lb)
controller.updateProjectileProperties(ProjectileProperties.heavyBall());

// Light balls (0.448 lb)
controller.updateProjectileProperties(ProjectileProperties.lightBall());

// Custom configuration
ProjectileProperties custom = new ProjectileProperties(
    0.14,   // diameter (meters)
    0.210   // mass (kg)
);
controller.updateProjectileProperties(custom);
```

### Logging Control

Toggle logging to reduce CPU usage:

```java
// Competition mode
controller.setLoggingEnabled(false);

// Practice mode
controller.setLoggingEnabled(true);
```

### Per-Shot Corrections

Adjust individual shots:

```java
CalibrationParameters adjusted = currentParams
    .withSpeedCorrectionFactor(0.98)  // 2% speed reduction
    .withSpinCorrectionFactor(1.05);  // 5% more spin

controller.updateCalibration(adjusted);
```

### Parameter Safety

All parameters are automatically constrained:

```java
// Constraint ranges:
// - Drag coefficient: 0.1 - 2.0
// - Speed efficiency: 0.5 - 1.0
// - Spin efficiency: 0.5 - 1.0
// - Magnus coefficient: 0.0 - 0.001
// - Restitution: 0.0 - 1.0
// - Friction: 0.0 - 1.0

// Extreme values are automatically clamped
CalibrationParameters safe = new CalibrationParameters()
    .withDragCoefficient(10.0);  // Clamped to 2.0
```

## Troubleshooting

### No solution found
- Robot exceeds maximum range to target
- Launch speed insufficient
- Check `canReachTarget()` return value

### Shots miss consistently
- Parameters require tuning (see Parameter Tuning section)
- Verify turret and hood angles are accurate
- Check shooter wheel speed consistency

### Logs not visible in AdvantageScope
- Confirm AdvantageKit is configured in robot code
- Verify `calculateAndLog()` is being called
- Check log file generation

## Further Reading

- [ADVANTAGEKIT_TUNING.md](ADVANTAGEKIT_TUNING.md) - Detailed calibration procedures
- [examples/wpilib/](examples/wpilib/) - Complete implementation examples  
- [WPILIB_INTEGRATION.md](WPILIB_INTEGRATION.md) - Advanced integration topics

## Support

- Review documentation files in this repository
- Open GitHub issue for bugs or feature requests
- Post on Chief Delphi with [TrajectoryCalc] tag

## API Reference

```java
// Calculate trajectory
var solution = trajectory.calculateAndLog(x, y, height, speed, spin);

// Check if reachable
boolean canHit = trajectory.getController().canReachTarget(x, y, height, speed);

// Log shot result
trajectory.logShotResult(x, y, solution, speed, spin, hit);

// Calibrate from logged data
trajectory.calibrate();

// Manual parameter update
trajectory.updateCalibration(dragCoeff, speedEfficiency);

// Get current parameters
CalibrationParameters params = trajectory.getCalibrationParameters();
```

That's it! You're ready to shoot from anywhere on the field.
