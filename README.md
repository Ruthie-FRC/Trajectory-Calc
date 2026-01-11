# Trajectory-Calc Trajectory Calculator

Production-grade inverse ballistic solver for FRC turret shooters. Calculates optimal launch angles from any field position using full physics modeling including drag, Magnus effect, collisions, and spin decay.

## Features
not all implemented yet bc the robot hasnt been designed but either have been or will be

- Full physics simulation: gravity, quadratic drag, Magnus effect, collision response, spin decay
- Inverse solver: automatically computes launch angles from target position
- SpinShotSimulator: exhaustive hooded-turret shot analysis utility
- ShooterAimingController: field-relative turret and hood control
- RK4 numerical integration with adaptive timestep near surfaces
- Shooter efficiency modeling with per-shot correction factors
- AdvantageKit logging integration for data-driven tuning
- Incremental calibration from logged shot data
- Pre-seeded optimization with position-based caching
- Real-time safe (deterministic, <100ms)
- Runtime-configurable ball properties and physics parameters

## Installation

### Via WPILib Vendor Dependencies

In VS Code with your robot project open:
1. Press Ctrl+Shift+P
2. Select "Manage Vendor Libraries"
3. Choose "Install new library (offline)"
4. Select `vendordeps/TrajectoryCalc.json` from this repository

### Basic Usage

```java
import frc.robot.trajectory.logging.advantagekit.AdvantageKitTrajectoryHelper;

public class ShooterSubsystem extends SubsystemBase {
    private final AdvantageKitTrajectoryHelper trajectory = 
        new AdvantageKitTrajectoryHelper("Shooter/Trajectory");
    
    public void shoot(Pose2d robotPose) {
        var solution = trajectory.calculateAndLog(
            robotPose.getX(), robotPose.getY(), 
            SHOOTER_HEIGHT, LAUNCH_SPEED, SPIN_RATE
        );
        
        setTurretAngle(solution.launchYawDeg);
        setHoodAngle(solution.launchPitchDeg);
        fire();
        
        boolean hit = checkIfScored();
        trajectory.logShotResult(
            robotPose.getX(), robotPose.getY(),
            solution, LAUNCH_SPEED, SPIN_RATE, hit
        );
    }
}
```

### AdvantageScope Integration

Trajectory data appears in AdvantageScope under `Shooter/Trajectory/`:
- Input parameters (position, velocity, spin)
- Output calculations (angles, risk scores)
- Shot results and outcomes
- Calibration parameters

### Calibration

```java
// Manual parameter adjustment
trajectory.updateCalibration(0.45, 0.92);

// Automatic calibration from logged shots
trajectory.calibrate();
```

## Key Capabilities

### AdvantageKit Teams

- Automatic logging to AdvantageScope for all calculations
- Data-driven calibration from match and practice logs
- Shot outcome tracking and analysis
- Rapid iteration without manual tuning

### General Use

- Field-position-independent shooting (no lookup tables required)
- Physics-based trajectories (not simplified parabolas)
- 5-20ms calculation time (real-time safe)
- Continuous learning from shot data
- Compatible with any WPILib robot (2024+)

## Installation

### Maven

```xml
<dependency>
    <groupId>com.ruthiefrc</groupId>
    <artifactId>trajectory-calc</artifactId>
    <version>1.0.0</version>
</dependency>
```

### Manual JAR

Download from releases or build from source:

```bash
git clone https://github.com/Ruthie-FRC/Trajectory-Calc.git
cd Trajectory-Calc
mvn clean package
```

Add `target/trajectory-calc-1.0.0.jar` to your robot project's `lib/` folder.

## Examples

See the [examples/wpilib/](examples/wpilib/) directory:

- `AdvantageKitShooterExample.java` - Complete AdvantageKit integration ⭐
- `ExampleShooterSubsystem.java` - Full-featured subsystem
- `AutoAimCommand.java` - Command-based example

## Building from Source

```bash
# Compile
mvn clean install

# Run tests
mvn test

# Package JAR
mvn package
```

## Support

- **AdvantageKit Integration**: See [ADVANTAGEKIT_TUNING.md](ADVANTAGEKIT_TUNING.md)
- **Issues**: https://github.com/Ruthie-FRC/Trajectory-Calc/issues
- **Discussions**: GitHub Discussions
- **Chief Delphi**: Post with [TrajectoryCalc] tag

## Requirements

- Java 11+
- current advantagekit (for robot integration)
- AdvantageKit (optional, for logging features)

## License

MIT License - Free for all FRC teams

## Credits

Developed for FRC robotics applications with AdvantageKit teams in mind. Physics model based on classical mechanics and empirical FRC game piece data.

Made by Ruthie-FRC, a sleep deprived freshman with an unhealthy physics obsession
