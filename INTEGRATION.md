# Trajectory Calculator - Production-Grade Ballistic Solver

A high-precision inverse ballistic solver for FRC robot shooters. Computes optimal launch angles to hit targets from any position on the field, accounting for drag, spin (Magnus effect), shooter inefficiencies, and collision physics.

## Features

- **Full Physics Model**: Gravity, quadratic drag, Magnus effect, collisions
- **Inverse Solver**: Automatically calculates launch angles from any position
- **RK4 Integration**: Accurate numerical integration with configurable timestep
- **Shooter Modeling**: Accounts for wheel slip, ball deformation, and efficiency losses
- **Calibration System**: Learn from real shots and improve accuracy over time
- **Zero External Dependencies**: Pure Java, works with any robot framework
- **Real-time Safe**: Deterministic, no allocations in hot path
- **Production Ready**: Comprehensive tests, clean API, documented

## Quick Start

### Maven Dependency

Add to your `pom.xml`:

```xml
<dependency>
    <groupId>com.ruthiefrc</groupId>
    <artifactId>trajectory-calc</artifactId>
    <version>1.0.0</version>
</dependency>
```

### Basic Usage

```java
import com.ruthiefrc.trajectory.*;

// Create shooter controller (one instance per robot)
ShooterController shooter = new ShooterController();

// Calculate shot from robot position (x, y, z in meters)
double robotX = 5.0;  // meters from target
double robotY = 2.0;
double robotZ = 0.8;  // shooter height
double launchSpeed = 12.0;  // m/s

InverseSolver.SolutionResult solution = shooter.calculateShot(
    robotX, robotY, robotZ, launchSpeed
);

if (solution.isHit()) {
    // Use the angles to aim
    double yaw = solution.launchYawDeg;
    double pitch = solution.launchPitchDeg;
    
    System.out.println("Aim: Yaw=" + yaw + "°, Pitch=" + pitch + "°");
    System.out.println("Quality: " + solution.score);
}
```

## Integration Examples

### WPILib Integration

```java
import com.ruthiefrc.trajectory.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterController trajectoryCalc;
    private final TalonFX flywheelMotor;
    private final TalonFX turretMotor;
    
    public ShooterSubsystem() {
        trajectoryCalc = new ShooterController();
        // ... initialize motors
    }
    
    public void aimAtTarget(Pose2d robotPose) {
        // Get solution from trajectory calculator
        InverseSolver.SolutionResult solution = trajectoryCalc.calculateShot(
            robotPose.getX(),
            robotPose.getY(),
            0.8,  // shooter height
            12.0  // launch speed
        );
        
        if (solution.isHit()) {
            // Set turret angle
            setTurretAngle(solution.launchYawDeg);
            setHoodAngle(solution.launchPitchDeg);
        }
    }
    
    public void logShot(Pose2d robotPose, boolean hit) {
        // Log for calibration
        InverseSolver.SolutionResult lastSolution = getLastSolution();
        trajectoryCalc.logShot(
            robotPose.getX(), robotPose.getY(), 0.8,
            lastSolution, 12.0, 100.0, hit
        );
    }
    
    // ... rest of subsystem
}
```

### Command Example

```java
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoAimCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DriveSubsystem drive;
    
    public AutoAimCommand(ShooterSubsystem shooter, DriveSubsystem drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();
        shooter.aimAtTarget(robotPose);
    }
    
    @Override
    public boolean isFinished() {
        return shooter.isAimed();
    }
}
```

### Non-WPILib Integration

Works with any Java robot framework:

```java
// In your main robot code
ShooterController shooter = new ShooterController();

// In your control loop
void updateShooting() {
    double x = getRobotX();
    double y = getRobotY();
    double z = getShooterHeight();
    
    InverseSolver.SolutionResult solution = shooter.calculateShot(x, y, z, 12.0);
    
    if (solution.isHit()) {
        setTurretAngle(solution.launchYawDeg);
        setHoodAngle(solution.launchPitchDeg);
        spinUpFlywheel(12.0);  // m/s
    }
}
```

## Advanced Configuration

### Custom Calibration Parameters

```java
// Create with custom physics parameters
CalibrationParameters customParams = new CalibrationParameters()
    .withDragCoefficient(0.45)        // Adjust for ball condition
    .withSpeedEfficiency(0.92)         // Your shooter's efficiency
    .withSpinEfficiency(0.88)          // Wheel grip efficiency
    .withRestitutionCoefficient(0.65); // Bounciness

ShooterController shooter = new ShooterController(customParams);
```

### Calibration from Real Data

```java
// Log shots during practice
shooter.logShot(x, y, z, solution, speed, spin, hit);

// After collecting data, calibrate
if (shooter.calibrate()) {
    System.out.println("Updated calibration: " + 
        shooter.getCalibrationStats());
}
```

### Target Position Configuration

If your target is not at origin:

```java
// Modify HubGeometry (see source for custom targets)
Vector3D targetCenter = new Vector3D(targetX, targetY, targetZ);
HubGeometry customHub = new HubGeometry(
    targetCenter,
    1.06,  // opening size
    1.83,  // height
    1.19   // footprint
);
```

## API Reference

### ShooterController

Main interface for trajectory calculations.

**Methods:**
- `calculateShot(x, y, z, speed)` - Calculate launch angles
- `calculateShot(x, y, z, speed, spin)` - With custom spin
- `simulateTrajectory(...)` - Test a specific trajectory
- `logShot(...)` - Log shot for calibration
- `calibrate()` - Update parameters from logged data
- `canReachTarget(x, y, z, maxSpeed)` - Quick feasibility check

### InverseSolver.SolutionResult

Returned by `calculateShot()`.

**Fields:**
- `launchYawDeg` - Horizontal angle (degrees)
- `launchPitchDeg` - Vertical angle (degrees)
- `score` - Quality score (higher = better)
- `trajectory` - Full simulated trajectory

**Methods:**
- `isHit()` - Returns true if solution hits target

### CalibrationParameters

Physics and shooter parameters.

**Parameters:**
- `dragCoefficient` - Air resistance (default: 0.47)
- `magnusCoefficient` - Spin effect (default: 0.00001)
- `speedEfficiency` - Launch speed loss (default: 0.95)
- `spinEfficiency` - Spin transfer (default: 0.90)
- `restitutionCoefficient` - Bounce (default: 0.6)
- `frictionCoefficient` - Friction (default: 0.3)

## Building from Source

```bash
# Clone repository
git clone https://github.com/Ruthie-FRC/Trajectory-Calc.git
cd Trajectory-Calc

# Build with Maven
mvn clean install

# Run tests
mvn test

# Create JAR
mvn package
```

The compiled JAR will be in `target/trajectory-calc-1.0.0.jar`.

## Performance

- **Calculation time**: ~5-20ms per shot (typical)
- **Memory**: No allocations in steady state
- **Deterministic**: Same inputs always produce same outputs
- **Thread-safe**: Create one instance per thread

## Physics Model

### Forces Modeled

1. **Gravity**: Standard 9.80665 m/s²
2. **Drag**: F = 0.5 × ρ × Cd × A × v²
3. **Magnus**: F = Cm × (ω × v)
4. **Collisions**: Restitution and friction

### Integration

- 4th-order Runge-Kutta (RK4)
- Default timestep: 1ms
- Adaptive timestep: Available via custom integrator

### Accuracy

- Typical error: < 5cm at 5m range (after calibration)
- Convergence tolerance: 1cm
- Maximum iterations: 50

## Troubleshooting

### "No solution found"

- Check if target is reachable: `canReachTarget()`
- Increase launch speed
- Verify robot position is correct

### "Poor accuracy"

- Calibrate with real shot data: `calibrate()`
- Adjust `speedEfficiency` parameter
- Check ball condition (worn balls have different drag)

### "Slow calculations"

- Reduce max iterations in InverseSolver
- Use feasibility check before full solve
- Increase convergence tolerance

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Add tests for new features
4. Submit pull request

## License

MIT License - See LICENSE file for details

## Support

- Issues: https://github.com/Ruthie-FRC/Trajectory-Calc/issues
- Documentation: https://github.com/Ruthie-FRC/Trajectory-Calc/wiki
- FRC Community: Chief Delphi forums

## Credits

Developed for FRC robotics applications. Physics model based on classical mechanics and empirical FRC game piece data.
