# Trajectory Calculator

Production-grade inverse ballistic solver for FRC robot shooters. Calculate optimal launch angles from any position on the field with full physics modeling.

## ✨ Perfect for AdvantageKit Teams!

This library is designed to integrate seamlessly with **AdvantageKit logging**, making it easy to tune your shooter from logged match/practice data.

## Quick Links

- 🚀 **[QUICKSTART.md](QUICKSTART.md)** - Get running in 5 minutes
- 📊 **[ADVANTAGEKIT_TUNING.md](ADVANTAGEKIT_TUNING.md)** - Tune from logs
- 🎯 **[SPIN_SHOT_SIMULATOR.md](SPIN_SHOT_SIMULATOR.md)** - Hooded-turret shot analysis (NEW)
- 🌀 **[SPIN_PHYSICS.md](SPIN_PHYSICS.md)** - Spin modeling guide
- 📖 **[WPILIB_INTEGRATION.md](WPILIB_INTEGRATION.md)** - Full integration guide

## Features

- ✅ Full physics: gravity, drag, Magnus effect, collisions, spin decay
- ✅ Inverse solver: automatically computes launch angles
- ✅ **SpinShotSimulator: exhaustive hooded-turret shot analysis (NEW)**
- ✅ RK4 numerical integration with adaptive timestep
- ✅ Shooter efficiency modeling with per-shot corrections
- ✅ **Automatic AdvantageKit logging for easy tuning**
- ✅ **Tune from robot logs - no manual testing needed!**
- ✅ Incremental calibration and pre-seeded optimization
- ✅ Calibration and learning system
- ✅ Zero dependencies - works with any robot framework
- ✅ Real-time safe and deterministic
- ✅ Runtime-configurable parameters

## Quick Start

### 1. Install via WPILib Vendor Deps

```bash
# In VS Code with your robot project open:
# Ctrl+Shift+P → "Manage Vendor Libraries" → "Install new library (offline)"
# Select: vendordeps/TrajectoryCalc.json from this repository
```

### 2. Add to Your Shooter Subsystem

```java
import com.ruthiefrc.trajectory.advantagekit.AdvantageKitTrajectoryHelper;

public class ShooterSubsystem extends SubsystemBase {
    // One line to set up trajectory calculator with AdvantageKit logging!
    private final AdvantageKitTrajectoryHelper trajectory = 
        new AdvantageKitTrajectoryHelper("Shooter/Trajectory");
    
    public void shoot(Pose2d robotPose) {
        // Calculate and automatically log everything
        var solution = trajectory.calculateAndLog(
            robotPose.getX(), robotPose.getY(), 
            SHOOTER_HEIGHT, LAUNCH_SPEED, SPIN_RATE
        );
        
        // Use the solution
        setTurretAngle(solution.launchYawDeg);
        setHoodAngle(solution.launchPitchDeg);
        fire();
        
        // After 1-2 seconds, log the result
        boolean hit = checkIfScored();
        trajectory.logShotResult(
            robotPose.getX(), robotPose.getY(),
            solution, LAUNCH_SPEED, SPIN_RATE, hit
        );
    }
}
```

### 3. View Logs in AdvantageScope

All your trajectory data automatically appears in AdvantageScope under `Shooter/Trajectory/`:
- Input parameters (robot position, speeds)
- Output calculations (angles, quality scores)
- Shot results (hit/miss)
- Calibration parameters

### 4. Tune from Logs

After practice, review your logs and tune:

```java
// Option 1: Manual tuning from log analysis
trajectory.updateCalibration(0.45, 0.92);  // drag, speed efficiency

// Option 2: Automatic tuning
trajectory.calibrate();  // Uses logged shot data
```

## Documentation

- **[ADVANTAGEKIT_TUNING.md](ADVANTAGEKIT_TUNING.md)** - Complete guide for tuning from AdvantageKit logs ⭐
- **[SPIN_SHOT_SIMULATOR.md](SPIN_SHOT_SIMULATOR.md)** - Software-only hooded-turret shot analysis ⭐ (NEW)
- **[SPIN_PHYSICS.md](SPIN_PHYSICS.md)** - Comprehensive spin modeling guide
- **[WPILIB_INTEGRATION.md](WPILIB_INTEGRATION.md)** - Full WPILib integration guide
- **[INTEGRATION.md](INTEGRATION.md)** - General usage and API reference
- **[examples/](examples/)** - Complete working examples

## Why This Library?

### For AdvantageKit Teams

- 📊 **Auto-logging**: Every calculation automatically logged to AdvantageScope
- 🎯 **Tune from data**: Use real match logs to calibrate, no guesswork
- 🔬 **Track everything**: See exactly what your shooter is doing
- 🚀 **Iterate fast**: Test changes, review logs, repeat

### For All Teams

- 🎮 **Shoot from anywhere**: No more lookup tables or hardcoded zones
- 🧮 **Real physics**: Drag, spin, collisions - not just parabolas
- ⚡ **Fast**: 5-20ms calculations, real-time safe
- 🔧 **Adaptable**: Learns and improves from real shots
- 📦 **Easy integration**: Works with any WPILib robot (2024+)

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
- WPILib 2024+ (for robot integration)
- AdvantageKit (optional, for logging features)

## License

MIT License - Free for all FRC teams

## Credits

Developed for FRC robotics applications with AdvantageKit teams in mind. Physics model based on classical mechanics and empirical FRC game piece data.
