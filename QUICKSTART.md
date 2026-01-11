# Quick Start Guide

Get up and running in 5 minutes!

## Step 1: Install (2 minutes)

### Download Vendordep

1. Go to this repository on GitHub
2. Download `vendordeps/TrajectoryCalc.json`

### Add to Your Robot

1. Open your robot project in VS Code
2. Press `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (Mac)
3. Type "WPILib: Manage Vendor Libraries"
4. Select "Install new library (offline)"
5. Navigate to the `TrajectoryCalc.json` you downloaded
6. Click "Select"

Done! The library is now part of your project.

## Step 2: Add to Subsystem (2 minutes)

Create or update your shooter subsystem:

```java
package frc.robot.subsystems;

import com.ruthiefrc.trajectory.*;
import com.ruthiefrc.trajectory.advantagekit.AdvantageKitTrajectoryHelper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // One line setup with runtime-configurable parameters!
    private final AdvantageKitTrajectoryHelper trajectory;
    
    // Your robot's constants
    private static final double SHOOTER_HEIGHT = 0.8;  // meters
    private static final double LAUNCH_SPEED = 12.0;   // m/s
    private static final double SPIN_RATE = 100.0;     // rad/s
    
    private InverseSolver.SolutionResult lastSolution;
    
    public ShooterSubsystem() {
        // Option 1: Use defaults
        trajectory = new AdvantageKitTrajectoryHelper("Shooter/Trajectory");
        
        // Option 2: Custom calibration from the start
        // CalibrationParameters params = new CalibrationParameters()
        //     .withSpeedEfficiency(0.92)
        //     .withDragCoefficient(0.45);
        // trajectory = new AdvantageKitTrajectoryHelper("Shooter/Trajectory", params);
    }
    
    public void calculateAim(Pose2d robotPose) {
        // Calculates angles + logs to AdvantageKit automatically
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
            // TODO: Replace with your actual hardware
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
    
    // NEW: Runtime configuration examples
    
    /**
     * Switch to worn ball configuration (lighter, smaller).
     */
    public void useWornBall() {
        trajectory.getController().updateProjectileProperties(
            ProjectileProperties.wornBall()
        );
    }
    
    /**
     * Disable logging during competition to reduce CPU load.
     */
    public void disableLogging() {
        trajectory.getController().setLoggingEnabled(false);
    }
    
    /**
     * Enable logging for practice/testing.
     */
    public void enableLogging() {
        trajectory.getController().setLoggingEnabled(true);
    }
}
```

## Step 3: Use in Commands (1 minute)

```java
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

## Step 4: View Logs

1. Enable your robot
2. Drive around and press your aim button
3. Open AdvantageScope
4. Load your log file
5. Look under `Shooter/Trajectory/`

You'll see:
- `Input/*` - Robot position, speeds
- `Output/*` - Calculated angles, predicted hit
- `Calibration/*` - Current tuning parameters

## Step 5: Tune Parameters

After practice, analyze your logs:

### Manual Tuning

If shots are falling short:
```java
// In your subsystem
public void tuneDrag() {
    CalibrationParameters newParams = trajectory.getCalibrationParameters()
        .withDragCoefficient(0.45)        // Decrease if shots short
        .withSpeedEfficiency(0.93);        // Increase if speed low
    
    trajectory.updateCalibration(newParams);
}
```

### Auto Tuning

After collecting 10+ shots with logged results:

```java
// Add this method to your subsystem
public void calibrate() {
    boolean updated = trajectory.calibrate();
    if (updated) {
        System.out.println("Calibration improved!");
        System.out.println(trajectory.getCalibrationStats());
    }
}
```

Call it in test mode or disabled after practice.

## Runtime Configuration Features (NEW)

### Incremental Calibration (NEW)

Enable real-time parameter learning during practice:

```java
// Enable incremental calibration for practice
controller.setIncrementalCalibrationEnabled(true);
controller.setLearningRate(0.05); // 5% adjustment per shot

// Parameters automatically update after each logged shot
controller.logShot(x, y, z, solution, speed, spin, hit);

// Disable during competition for stability
controller.setIncrementalCalibrationEnabled(false);
```

**How it works:**
- After each logged shot, system analyzes recent performance
- If hit rate drops below 60%, adjusts drag and speed efficiency
- Learning rate controls adaptation speed (0.0-1.0, default: 0.05)
- Requires 5 baseline shots before incremental updates begin
- Changes are applied automatically, no manual calibration needed

### Pre-Seeded Guesses (NEW)

Solver uses previous successful shots for faster convergence:

```java
// First shot - uses geometric estimate
var solution1 = controller.calculateShot(3.0, 0.0, 0.5, 12.0, 100.0);

// Second shot from nearby position - uses cached angles
var solution2 = controller.calculateShot(3.1, 0.1, 0.5, 12.0, 100.0);
// Converges faster with better initial guess

// Control pre-seeding (enabled by default)
// Note: Access through solver if needed for custom control
```

**Benefits:**
- Faster convergence for repeated shots
- More consistent solutions
- Reduces computation time by 10-30%
- Automatically caches successful shots on 0.5m grid

### Ball Condition

Switch configurations based on ball wear:

```java
// For worn/compressed balls (lighter, smaller)
controller.updateProjectileProperties(ProjectileProperties.wornBall());

// For heavy balls (0.500 lb)
controller.updateProjectileProperties(ProjectileProperties.heavyBall());

// For light balls (0.448 lb)
controller.updateProjectileProperties(ProjectileProperties.lightBall());

// Custom ball properties
ProjectileProperties custom = new ProjectileProperties(
    0.14,   // diameter in meters (slightly compressed)
    0.210   // mass in kg
);
controller.updateProjectileProperties(custom);
```

### Logging Control

Reduce CPU load during competition:

```java
// Disable logging during competition
controller.setLoggingEnabled(false);

// Enable for practice
controller.setLoggingEnabled(true);
```

### Per-Shot Corrections

Fine-tune individual shots:

```java
CalibrationParameters adjusted = currentParams
    .withSpeedCorrectionFactor(0.98)  // 2% speed reduction this shot
    .withSpinCorrectionFactor(1.05);  // 5% more spin this shot

controller.updateCalibration(adjusted);
```

### Safety Limits

All parameters are automatically clamped to safe ranges:

```java
// These values are automatically constrained:
// - Drag coefficient: 0.1 - 2.0
// - Speed efficiency: 0.5 - 1.0
// - Spin efficiency: 0.5 - 1.0
// - Magnus coefficient: 0.0 - 0.001
// - Restitution: 0.0 - 1.0
// - Friction: 0.0 - 1.0

// Extreme values are clamped automatically
CalibrationParameters safe = new CalibrationParameters()
    .withDragCoefficient(10.0);  // Clamped to 2.0
// No need to manually validate!
```

## Common Issues

### "No solution found"
- Robot too far from target
- Launch speed too low
- Check if `canReachTarget()` returns true

### "Shots miss consistently"
- Need to tune parameters (see Step 5)
- Check if turret/hood angles are accurate
- Verify shooter wheel speed is consistent

### "Not seeing logs in AdvantageScope"
- Make sure AdvantageKit is set up in your robot
- Check that you're calling `calculateAndLog()`
- Verify logs are being saved

## Next Steps

- **Read**: [ADVANTAGEKIT_TUNING.md](ADVANTAGEKIT_TUNING.md) for detailed tuning guide
- **See**: [examples/wpilib/](examples/wpilib/) for complete examples
- **Learn**: [WPILIB_INTEGRATION.md](WPILIB_INTEGRATION.md) for advanced features

## Need Help?

- Check the documentation files in this repository
- Open an issue on GitHub
- Post on Chief Delphi with [TrajectoryCalc] tag

## Quick Reference

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

That's it! You're ready to shoot from anywhere on the field! 🎯
