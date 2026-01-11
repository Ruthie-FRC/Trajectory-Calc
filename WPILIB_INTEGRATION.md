# WPILib Installation & Integration Guide

## Quick Start - Adding to Your Robot Project

### Method 1: Vendor Dependency (Recommended)

1. **Download the vendor JSON file** from this repository:
   - Download `vendordeps/TrajectoryCalc.json` from this repo

2. **Install in your WPILib project**:
   - Open VS Code with your robot project
   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
   - Type "Manage Vendor Libraries"
   - Select "Install new library (offline)"
   - Navigate to the downloaded `TrajectoryCalc.json`
   - Select it and click OK

3. **Build your project**:
   ```bash
   ./gradlew build
   ```

The library is now available in your robot code!

### Method 2: Manual JAR Installation

1. **Build or download the JAR**:
   ```bash
   git clone https://github.com/Ruthie-FRC/Trajectory-Calc.git
   cd Trajectory-Calc
   mvn clean package
   ```

2. **Copy to your robot project**:
   ```bash
   cp target/trajectory-calc-1.0.0.jar ~/frc/YourRobotProject/lib/
   ```

3. **Add to build.gradle**:
   ```gradle
   dependencies {
       implementation files('lib/trajectory-calc-1.0.0.jar')
   }
   ```

## Integration Examples

### Basic Subsystem Integration

```java
import com.ruthiefrc.trajectory.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterController trajectoryCalc;
    
    public ShooterSubsystem() {
        trajectoryCalc = new ShooterController();
    }
    
    public void aimAtTarget(Pose2d robotPose) {
        InverseSolver.SolutionResult solution = trajectoryCalc.calculateShot(
            robotPose.getX(),
            robotPose.getY(),
            0.8,  // shooter height
            12.0  // launch speed m/s
        );
        
        if (solution.isHit()) {
            setTurretAngle(solution.launchYawDeg);
            setHoodAngle(solution.launchPitchDeg);
        }
    }
}
```

### With AdvantageKit Logging

```java
import com.ruthiefrc.trajectory.*;
import com.ruthiefrc.trajectory.advantagekit.TrajectoryLogger;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterController trajectoryCalc;
    private TrajectoryLogger.Inputs inputs = new TrajectoryLogger.Inputs();
    private TrajectoryLogger.Outputs outputs = new TrajectoryLogger.Outputs();
    
    public void aimAtTarget(Pose2d robotPose) {
        // Create inputs
        inputs = TrajectoryLogger.createInputs(
            robotPose.getX(), robotPose.getY(), 0.8,
            0.0, 0.0, 1.83,  // target position
            12.0, 100.0      // speed and spin
        );
        
        // Calculate
        long startTime = System.currentTimeMillis();
        InverseSolver.SolutionResult solution = trajectoryCalc.calculateShot(
            robotPose.getX(), robotPose.getY(), 0.8, 12.0
        );
        long calcTime = System.currentTimeMillis() - startTime;
        
        // Create outputs
        outputs = TrajectoryLogger.toOutputs(solution, calcTime);
        
        // Log to AdvantageKit
        Logger.recordOutput("Shooter/Trajectory/Input/RobotX", inputs.robotX);
        Logger.recordOutput("Shooter/Trajectory/Input/RobotY", inputs.robotY);
        Logger.recordOutput("Shooter/Trajectory/Output/Yaw", outputs.launchYawDeg);
        Logger.recordOutput("Shooter/Trajectory/Output/Pitch", outputs.launchPitchDeg);
        Logger.recordOutput("Shooter/Trajectory/Output/IsHit", outputs.isHit);
        Logger.recordOutput("Shooter/Trajectory/Output/CalcTime", outputs.calculationTimeMs);
    }
}
```

### Command-Based Robot

```java
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAimCommand extends Command {
    private final ShooterSubsystem shooter;
    private final DriveSubsystem drive;
    
    public AutoAimCommand(ShooterSubsystem shooter, DriveSubsystem drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        shooter.aimAtTarget(drive.getPose());
    }
    
    @Override
    public boolean isFinished() {
        return shooter.isAimed();
    }
}

// In RobotContainer:
public RobotContainer() {
    driver.y().whileTrue(new AutoAimCommand(shooter, drive));
}
```

## Advanced Features

### Custom Calibration

```java
CalibrationParameters params = new CalibrationParameters()
    .withDragCoefficient(0.45)        // Adjust for ball wear
    .withSpeedEfficiency(0.92)         // Your shooter's efficiency
    .withSpinEfficiency(0.88)          // Wheel grip
    .withRestitutionCoefficient(0.65); // Ball bounciness

ShooterController controller = new ShooterController(params);
```

### Shot Logging and Auto-Calibration

```java
// Log each shot
public void shoot() {
    Pose2d pose = drive.getPose();
    InverseSolver.SolutionResult solution = getLastSolution();
    
    // Fire the shot
    fire();
    
    // After 1-2 seconds, check if it scored (vision, sensors, etc.)
    boolean hit = checkIfScored();
    
    // Log for calibration
    controller.logShot(pose.getX(), pose.getY(), 0.8, 
                      solution, 12.0, 100.0, hit);
}

// Calibrate after practice or autonomous
public void calibrate() {
    if (controller.calibrate()) {
        System.out.println("Calibration updated!");
        System.out.println(controller.getCalibrationStats());
    }
}
```

### NetworkTables Integration

```java
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterSubsystem extends SubsystemBase {
    private final NetworkTable table;
    
    public ShooterSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("Trajectory");
    }
    
    @Override
    public void periodic() {
        // Publish solution to NT
        if (lastSolution != null) {
            table.getEntry("LaunchYaw").setDouble(lastSolution.launchYawDeg);
            table.getEntry("LaunchPitch").setDouble(lastSolution.launchPitchDeg);
            table.getEntry("IsHit").setBoolean(lastSolution.isHit());
            table.getEntry("Score").setDouble(lastSolution.score);
        }
    }
}
```

## Tuning Parameters

### Finding Your Drag Coefficient

1. Record 10-20 shots with known robot positions
2. Note which shots were short/long
3. Adjust `dragCoefficient`:
   - Shots falling short → decrease drag (e.g., 0.47 → 0.45)
   - Shots going long → increase drag (e.g., 0.47 → 0.49)

### Finding Your Speed Efficiency

1. Measure actual flywheel RPM vs commanded
2. Convert to m/s: `speed = (RPM * wheel_circumference) / 60`
3. Calculate efficiency: `efficiency = actual_speed / commanded_speed`
4. Set `speedEfficiency` to this value

### Example Tuning Session

```java
// Start with defaults
CalibrationParameters params = new CalibrationParameters();

// Test shots - if consistently short:
params = params.withDragCoefficient(0.44);

// Test shots - if still need adjustment:
params = params.withSpeedEfficiency(0.93);

// Apply to controller
controller.updateCalibration(params);
```

## Troubleshooting

### "No solution found"
- Check if target is in range: `controller.canReachTarget(...)`
- Increase launch speed
- Verify robot position is correct

### "Shots are inaccurate"
- Calibrate with real data: `controller.calibrate()`
- Check ball condition (worn balls have different drag)
- Verify shooter wheel speed is consistent

### "Calculations are slow"
- Normal calculation time: 5-20ms
- If slower, check CPU usage
- Consider caching solutions for same positions

### "Build errors with vendordep"
- Ensure WPILib 2024 or later
- Clean build: `./gradlew clean build`
- Check TrajectoryCalc.json is in `vendordeps/` folder

## Performance Tips

1. **Cache calculations**: Don't recalculate every loop if robot hasn't moved
2. **Pre-check feasibility**: Use `canReachTarget()` before full solve
3. **Reduce max iterations**: For faster (less accurate) solutions
4. **Log selectively**: Only log when actually shooting, not continuously

## Example Complete Integration

See `examples/wpilib/ExampleShooterSubsystem.java` for a complete, production-ready example with:
- AdvantageKit logging
- NetworkTables support
- Shot logging
- Auto-calibration
- Dashboard integration

## Support

- **Issues**: https://github.com/Ruthie-FRC/Trajectory-Calc/issues
- **Discussions**: GitHub Discussions
- **Chief Delphi**: Post in Programming forum with [TrajectoryCalc] tag
