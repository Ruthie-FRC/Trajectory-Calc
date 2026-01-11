# Trajectory Calculator

Production-grade inverse ballistic solver for FRC robot shooters. Calculate optimal launch angles from any position on the field with full physics modeling.

## Features

- ✅ Full physics: gravity, drag, Magnus effect, collisions
- ✅ Inverse solver: automatically computes launch angles
- ✅ RK4 numerical integration
- ✅ Shooter efficiency modeling
- ✅ Calibration and learning system
- ✅ Zero dependencies - works with any robot framework
- ✅ Real-time safe and deterministic

## Quick Start

```java
import com.ruthiefrc.trajectory.*;

// Create controller
ShooterController shooter = new ShooterController();

// Calculate shot
InverseSolver.SolutionResult solution = shooter.calculateShot(
    robotX, robotY, robotZ, launchSpeed
);

// Use the angles
if (solution.isHit()) {
    setTurretAngle(solution.launchYawDeg);
    setHoodAngle(solution.launchPitchDeg);
}
```

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
mvn clean package
```

Add `target/trajectory-calc-1.0.0.jar` to your robot project.

## Integration

See [INTEGRATION.md](INTEGRATION.md) for detailed integration guides including:
- WPILib integration examples
- Command-based robot examples  
- Non-WPILib frameworks
- Custom calibration
- Performance tuning

## Building

```bash
# Compile
mvn clean install

# Run tests
mvn test

# Package JAR
mvn package
```

## Documentation

- **Integration Guide**: [INTEGRATION.md](INTEGRATION.md)
- **API Documentation**: Generate with `mvn javadoc:javadoc`

## License

MIT License - Free for all FRC teams
