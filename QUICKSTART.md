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

import com.ruthiefrc.trajectory.advantagekit.AdvantageKitTrajectoryHelper;
import com.ruthiefrc.trajectory.InverseSolver;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // One line setup!
    private final AdvantageKitTrajectoryHelper trajectory = 
        new AdvantageKitTrajectoryHelper("Shooter/Trajectory");
    
    // Your robot's constants
    private static final double SHOOTER_HEIGHT = 0.8;  // meters
    private static final double LAUNCH_SPEED = 12.0;   // m/s
    private static final double SPIN_RATE = 100.0;     // rad/s
    
    private InverseSolver.SolutionResult lastSolution;
    
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
// In your subsystem constructor
private final AdvantageKitTrajectoryHelper trajectory = 
    new AdvantageKitTrajectoryHelper("Shooter/Trajectory",
        new CalibrationParameters()
            .withSpeedEfficiency(0.92)    // Decrease if short
            .withDragCoefficient(0.45)     // Decrease if short at range
    );
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
