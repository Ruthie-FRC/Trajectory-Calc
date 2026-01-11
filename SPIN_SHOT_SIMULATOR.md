# SpinShotSimulator Usage Guide

## Overview

`SpinShotSimulator` is a software-only testing utility for comprehensive hooded-turret shot analysis. It simulates all possible turret orientations to find the optimal trajectory while accounting for full physics including drag, Magnus effect, spin decay, collisions, and bounce-out risk.

## Key Features

- **Exhaustive Search**: Tests all turret angles within configurable yaw/pitch ranges
- **Full Physics**: Drag, Magnus effect, spin decay, rim collisions, energy loss
- **Multi-Factor Scoring**: Same algorithm as InverseSolver (bounce-out risk + rim proximity)
- **Comprehensive Reports**: Trajectory over time, spin decay, apex, flight time, entry angle
- **Perturbation Testing**: Multiple trials with speed/spin variations to test consistency
- **AdvantageKit Logging**: Optional logging to AdvantageScope with zero dependencies
- **Deterministic**: Same inputs always produce same outputs
- **Real-Time Analysis**: Typical simulations complete in 1-4 seconds

## Basic Usage

### Simple Simulation

```java
import com.ruthiefrc.trajectory.*;

// Create simulator with default parameters
SpinShotSimulator simulator = new SpinShotSimulator();

// Define target and initial conditions
Vector3D target = new Vector3D(3.0, 0.0, 2.5);  // x, y, z (meters)
double speed = 12.0;  // m/s
Vector3D spin = new Vector3D(0, 200, 0);  // Backspin around Y-axis (rad/s)

// Run simulation
SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);

// Print report
System.out.println(result.generateReport());
```

**Output:**
```
=== Spin Shot Simulation Report ===
Simulations Run: 117
Best Launch Angles: Yaw=0.00°, Pitch=35.50°
Risk Score: 2.345
Predicted Outcome: HIT
Apex Height: 3.42 m
Flight Time: 0.823 s
Entry Angle: 42.3°
Initial Spin: 200.0 rad/s
Final Spin: 182.4 rad/s
Spin Decay: 8.8%
Trajectory Points: 824
===================================
```

### Configure Turret Ranges

```java
// Set custom yaw range
simulator.setYawRange(-90, 90, 10);  // min, max, step (degrees)

// Set custom pitch range
simulator.setPitchRange(15, 75, 5);  // min, max, step (degrees)

// Now simulations will only search within these ranges
SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
```

**Notes:**
- Smaller step sizes = more precise but slower
- Typical: yaw step 5-10°, pitch step 2-5°
- For quick testing: larger steps (15°, 10°)
- For final analysis: smaller steps (2°, 1°)

## Advanced Features

### Perturbation Testing

Test consistency and deterministic behavior with multiple trials:

```java
// Run 10 trials with speed ±0.5 m/s and spin ±20 rad/s variations
SpinShotSimulator.SimulationResult[] results = simulator.simulateShotWithPerturbations(
    target, 
    12.0,                    // Nominal speed
    new Vector3D(0, 200, 0), // Nominal spin
    10,                      // Number of trials
    0.5,                     // Speed perturbation (m/s)
    20.0                     // Spin perturbation (rad/s)
);

// Analyze results
for (int i = 0; i < results.length; i++) {
    System.out.printf("Trial %d: Score=%.3f, Hit=%s, Yaw=%.1f°, Pitch=%.1f°\n",
        i, results[i].bestScore, results[i].hit, 
        results[i].launchYawDeg, results[i].launchPitchDeg);
}
```

**Use Cases:**
- Validate solver robustness to sensor noise
- Test shooter wheel variance
- Verify deterministic behavior
- Estimate success rate under perturbations

### Custom Ball Properties

```java
// Test with worn ball
ProjectileProperties wornBall = ProjectileProperties.wornBall();
SpinShotSimulator simulator = new SpinShotSimulator(new CalibrationParameters(), wornBall);

// Or custom properties
ProjectileProperties customBall = new ProjectileProperties(0.22, 0.11);  // mass(kg), diameter(m)
simulator = new SpinShotSimulator(new CalibrationParameters(), customBall);

SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
```

### Custom Calibration Parameters

```java
// Create custom parameters
CalibrationParameters params = new CalibrationParameters()
    .withDragCoefficient(0.5)
    .withMagnusCoefficient(0.00002)
    .withSpinDecayRate(0.08)
    .withSpeedEfficiency(0.95)
    .withSpinEfficiency(0.93);

SpinShotSimulator simulator = new SpinShotSimulator(params, new ProjectileProperties());
SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);
```

## AdvantageKit Integration

### Enable Logging

```java
// Simulate with logging
SpinShotSimulator.SimulationResult result = simulator.simulateShot(
    target, 
    speed, 
    spin,
    "Shooter/Simulation/"  // AdvantageKit log prefix
);

// Logs appear in AdvantageScope under:
// Shooter/Simulation/Input/TargetX, TargetY, TargetZ
// Shooter/Simulation/Input/Speed, SpinRate, SpinAxisX, SpinAxisY, SpinAxisZ
// Shooter/Simulation/Output/LaunchYaw, LaunchPitch, RiskScore, Hit, Outcome
// Shooter/Simulation/Metrics/ApexHeight, FlightTime, EntryAngle, SimulationsRun, CalculationTimeMs
// Shooter/Simulation/Spin/InitialRate, FinalRate, DecayPercent
```

### Subsystem Integration

```java
public class ShooterAnalysisSubsystem extends SubsystemBase {
    private final SpinShotSimulator simulator;
    
    public ShooterAnalysisSubsystem() {
        this.simulator = new SpinShotSimulator();
        simulator.setYawRange(-90, 90, 10);
        simulator.setPitchRange(20, 70, 5);
    }
    
    @Override
    public void periodic() {
        // Get current target
        Vector3D target = getTargetPosition();
        
        // Simulate with current parameters
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(
            target,
            getCurrentSpeed(),
            getCurrentSpin(),
            "Shooter/Analysis/"  // Log to AdvantageScope
        );
        
        // Use result for visualization or analysis
        if (result.hit) {
            setIndicator("READY", result.bestScore);
        } else {
            setIndicator("OUT OF RANGE", result.bestScore);
        }
    }
}
```

## Accessing Results

### SimulationResult Fields

```java
SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);

// Basic info
boolean hit = result.hit;
double yaw = result.launchYawDeg;
double pitch = result.launchPitchDeg;
double score = result.bestScore;
String outcome = result.predictedOutcome;  // "HIT", "HIT_WITH_BOUNCE_RISK", "MISS"

// Trajectory metrics
double apex = result.apexHeight;      // meters
double time = result.flightTime;      // seconds
double angle = result.entryAngleDeg;  // degrees

// Spin decay
double initialSpin = result.spinDecay.getInitialSpinRate();
double finalSpin = result.spinDecay.getFinalSpinRate();
double decay = result.spinDecay.getDecayPercentage();

// Performance
int simCount = result.simulationsRun;
```

### Trajectory Over Time

```java
// Access full trajectory data
SpinShotSimulator.TrajectoryOverTime trajectory = result.trajectory;

for (int i = 0; i < trajectory.times.length; i++) {
    double t = trajectory.times[i];
    Vector3D pos = trajectory.positions[i];
    Vector3D vel = trajectory.velocities[i];
    Vector3D spin = trajectory.spins[i];
    
    System.out.printf("t=%.3f: pos=%s, vel=%s, spin=%.1f rad/s\n",
        t, pos, vel, spin.magnitude());
}
```

### Spin Decay Curve

```java
SpinShotSimulator.SpinDecayCurve spinCurve = result.spinDecay;

// Plot or analyze spin over time
for (int i = 0; i < spinCurve.times.length; i++) {
    System.out.printf("t=%.3f: spin=%.1f rad/s\n",
        spinCurve.times[i], spinCurve.spinRates[i]);
}
```

### Export Options

```java
// Human-readable report
String report = result.generateReport();
System.out.println(report);

// JSON format for external tools
String json = result.toJSON();
saveToFile("simulation_result.json", json);
```

## Use Cases

### 1. Field Position Analysis

Simulate from multiple field positions to map shooting range:

```java
for (double x = 1.0; x <= 6.0; x += 0.5) {
    for (double y = -3.0; y <= 3.0; y += 0.5) {
        Vector3D robotPos = new Vector3D(x, y, 0.5);  // Shooter height
        Vector3D target = new Vector3D(3.0, 0.0, 2.5);  // HUB center
        
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(
            target, 12.0, new Vector3D(0, 200, 0)
        );
        
        System.out.printf("(%1f, %.1f): %s (score=%.2f)\n",
            x, y, result.hit ? "HIT" : "MISS", result.bestScore);
    }
}
```

### 2. Parameter Tuning

Find optimal speed and spin:

```java
double bestScore = Double.NEGATIVE_INFINITY;
double bestSpeed = 0;
double bestSpinRate = 0;

for (double speed = 8.0; speed <= 16.0; speed += 0.5) {
    for (double spinRate = 0; spinRate <= 300; spinRate += 25) {
        SpinShotSimulator.SimulationResult result = simulator.simulateShot(
            target, speed, new Vector3D(0, spinRate, 0)
        );
        
        if (result.bestScore > bestScore) {
            bestScore = result.bestScore;
            bestSpeed = speed;
            bestSpinRate = spinRate;
        }
    }
}

System.out.printf("Optimal: speed=%.1f m/s, spin=%.0f rad/s, score=%.3f\n",
    bestSpeed, bestSpinRate, bestScore);
```

### 3. Spin Axis Optimization

Test different spin axes:

```java
Vector3D[] spinAxes = {
    new Vector3D(0, 1, 0),      // Pure backspin
    new Vector3D(1, 0, 0),      // Pure sidespin
    new Vector3D(0, 0, 1),      // Pure topspin
    new Vector3D(0.7, 0.7, 0),  // Diagonal
    new Vector3D(0.577, 0.577, 0.577)  // 3D spin
};

for (Vector3D axis : spinAxes) {
    Vector3D spin = axis.scale(200);  // 200 rad/s along axis
    SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, 12.0, spin);
    
    System.out.printf("Axis %s: score=%.3f, decay=%.1f%%\n",
        axis, result.bestScore, result.spinDecay.getDecayPercentage());
}
```

### 4. Turret Range Validation

Verify turret can reach target:

```java
// Set turret physical limits
simulator.setYawRange(-120, 120, 5);  // Physical yaw limits
simulator.setPitchRange(10, 80, 3);   // Physical pitch limits

SpinShotSimulator.SimulationResult result = simulator.simulateShot(target, speed, spin);

if (result.hit) {
    System.out.printf("Target reachable: yaw=%.1f°, pitch=%.1f°\n",
        result.launchYawDeg, result.launchPitchDeg);
} else {
    System.out.println("Target out of turret range!");
}
```

## Performance Tips

### Speed vs. Precision Trade-offs

```java
// Fast testing (coarse search)
simulator.setYawRange(-45, 45, 15);   // 7 yaw angles
simulator.setPitchRange(20, 60, 10);  // 5 pitch angles
// Total: 35 simulations (~0.5 seconds)

// Balanced (default)
simulator.setYawRange(-45, 45, 10);   // 10 yaw angles
simulator.setPitchRange(20, 60, 5);   // 9 pitch angles
// Total: 90 simulations (~1 second)

// High precision
simulator.setYawRange(-45, 45, 2);    // 46 yaw angles
simulator.setPitchRange(20, 60, 1);   // 41 pitch angles
// Total: 1886 simulations (~15 seconds)
```

### Parallel Batch Processing

For multiple targets, consider parallel execution:

```java
List<Vector3D> targets = Arrays.asList(
    new Vector3D(2.0, 0.0, 2.0),
    new Vector3D(3.0, 1.0, 2.5),
    new Vector3D(4.0, -1.0, 2.8)
);

// Process in parallel (requires external threading)
targets.parallelStream()
    .map(target -> simulator.simulateShot(target, 12.0, new Vector3D(0, 200, 0)))
    .forEach(result -> System.out.println(result.generateReport()));
```

## Troubleshooting

### No Valid Trajectory Found

```
result.hit = false
result.bestScore = -100.0
```

**Possible causes:**
- Target out of range for given speed
- Turret angle limits too restrictive
- Speed too low/high for target distance
- Spin causing excessive Magnus deflection

**Solutions:**
```java
// Try wider angle ranges
simulator.setYawRange(-180, 180, 10);
simulator.setPitchRange(10, 80, 5);

// Try different speeds
for (double speed = 8.0; speed <= 20.0; speed += 1.0) {
    result = simulator.simulateShot(target, speed, spin);
    if (result.hit) break;
}
```

### High Bounce-Out Risk

```
result.predictedOutcome = "HIT_WITH_BOUNCE_RISK"
result.bestScore < 0
```

**Possible causes:**
- Shallow entry angle
- Insufficient downward velocity
- Spin not aligned with entry

**Solutions:**
```java
// Increase backspin
Vector3D higherSpin = new Vector3D(0, 300, 0);

// Try lower speed for steeper angle
result = simulator.simulateShot(target, 10.0, higherSpin);
```

### Slow Simulations

**Causes:**
- Too many angle combinations
- Very fine angle steps

**Solutions:**
```java
// Coarsen step sizes
simulator.setYawRange(-45, 45, 15);  // Larger steps
simulator.setPitchRange(20, 60, 10);

// Or narrow search range
simulator.setYawRange(-30, 30, 5);   // Smaller range
simulator.setPitchRange(25, 55, 3);
```

## Best Practices

1. **Start Coarse, Then Refine**: Use large steps for initial testing, then narrow range and decrease step size
2. **Validate with Perturbations**: Always run perturbation tests to ensure solutions are robust
3. **Log to AdvantageKit**: Use logging during testing to visualize trajectories in AdvantageScope
4. **Check Predicted Outcome**: Don't just check `hit` - also verify `predictedOutcome` is not "HIT_WITH_BOUNCE_RISK"
5. **Export Results**: Save JSON reports for offline analysis and comparison
6. **Batch Analysis**: Run simulations for multiple field positions to create shooting range maps

## API Reference

### Constructor

```java
SpinShotSimulator()
SpinShotSimulator(CalibrationParameters params, ProjectileProperties props)
```

### Configuration Methods

```java
void setYawRange(double minDeg, double maxDeg, double stepDeg)
void setPitchRange(double minDeg, double maxDeg, double stepDeg)
```

### Simulation Methods

```java
SimulationResult simulateShot(Vector3D target, double speed, Vector3D spin)
SimulationResult simulateShot(Vector3D target, double speed, Vector3D spin, String logPrefix)
SimulationResult[] simulateShotWithPerturbations(Vector3D target, double speed, Vector3D spin,
                                                 int numTrials, double speedPerturbation, 
                                                 double spinPerturbation)
```

### Result Classes

```java
class SimulationResult {
    boolean hit;
    double launchYawDeg, launchPitchDeg;
    double bestScore;
    String predictedOutcome;
    double apexHeight, flightTime, entryAngleDeg;
    SpinDecayCurve spinDecay;
    TrajectoryOverTime trajectory;
    int simulationsRun;
    
    String generateReport();
    String toJSON();
}

class SpinDecayCurve {
    double[] times, spinRates;
    double getInitialSpinRate();
    double getFinalSpinRate();
    double getDecayPercentage();
}

class TrajectoryOverTime {
    double[] times;
    Vector3D[] positions, velocities, spins;
}
```

## Examples Repository

Complete working examples can be found in `examples/standalone/SpinShotSimulatorExamples.java` (to be added).

## Support

For questions or issues:
1. Check test files: `SpinShotSimulatorTest.java` for usage examples
2. Review main documentation: `SPIN_PHYSICS.md`
3. Check AdvantageKit integration: `ADVANTAGEKIT_TUNING.md`
