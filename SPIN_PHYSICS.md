# Spin Physics for Foam Balls

Comprehensive guide to spin simulation, Magnus effects, and bounce-out prevention in the TrajectoryCalc library.

## Overview

The TrajectoryCalc library models the complete physics of foam ball spin, including:
- **Magnus force** from ball rotation
- **Spin decay** over time due to air resistance
- **Bounce interactions** with spin-induced velocity changes
- **Bounce-out risk** evaluation based on spin alignment

This enables realistic trajectory prediction and helps optimize shots to minimize bounce-out probability.

---

## Spin Vector Control

### Basic Usage (Backspin)

```java
// Default: backspin around Y-axis
InverseSolver.SolutionResult solution = controller.calculateShot(
    3.0, 0.0, 1.0,  // Robot position (x, y, shooter height)
    12.0,            // Launch speed (m/s)
    200.0            // Spin rate (rad/s)
);
```

### Full Spin Axis Control

```java
// Specify custom spin axis
InverseSolver.SolutionResult solution = controller.calculateShot(
    3.0, 0.0, 1.0,   // Robot position
    12.0,             // Launch speed
    200.0,            // Spin rate (rad/s)
    0.0, 1.0, 0.0    // Spin axis: X, Y, Z (normalized automatically)
);
```

### Spin Axis Examples

```java
// Backspin (typical for shooters)
calculateShot(x, y, z, speed, spinRate, 0, 1, 0);

// Sidespin right
calculateShot(x, y, z, speed, spinRate, 1, 0, 0);

// Sidespin left
calculateShot(x, y, z, speed, spinRate, -1, 0, 0);

// Topspin (rare, increases bounce-out risk)
calculateShot(x, y, z, speed, spinRate, 0, 0, 1);

// Diagonal spin
calculateShot(x, y, z, speed, spinRate, 0.707, 0.707, 0);
```

---

## Magnus Effect

### Physics Model

The Magnus force creates lift perpendicular to both velocity and spin:

```
F_magnus = Cm × |ω| × |v| × (ω × v)
```

Where:
- `Cm` = Magnus coefficient (tunable, default 0.00001)
- `ω` = Spin vector (rad/s)
- `v` = Velocity vector (m/s)

### Practical Effects

**Backspin (Y-axis):**
- Creates upward lift, extending range
- Flattens trajectory
- Reduces bounce-out when aligned with downward entry

**Sidespin (X-axis):**
- Creates lateral deviation
- Can help avoid obstacles
- May increase rim contact risk

**Topspin (Z-axis):**
- Creates downward force
- Steeper trajectory
- **Warning**: Increases bounce-out risk significantly

---

## Spin Decay

Foam balls lose spin over time due to air resistance.

### Decay Model

```java
// Spin decay rate increases with ball velocity
double spinDecayFactor = spinDecayRate × (1.0 + velocity / 20.0);
dω/dt = -spinDecayFactor × ω
```

### Configuration

```java
// Adjust spin decay rate
CalibrationParameters params = new CalibrationParameters()
    .withSpinDecayRate(0.08);  // Higher = faster decay (default: 0.05)

controller.updateCalibration(params);
```

### Typical Behavior

| Spin Rate | After 0.5s | After 1.0s | After 2.0s |
|-----------|------------|------------|------------|
| 500 rad/s | ~450 rad/s | ~400 rad/s | ~320 rad/s |
| 200 rad/s | ~180 rad/s | ~160 rad/s | ~130 rad/s |
| 100 rad/s | ~90 rad/s  | ~80 rad/s  | ~65 rad/s  |

---

## Bounce & Rim Interactions

### Spin Effects on Bounces

When the ball contacts the HUB rim or surfaces:

1. **Spin reduction** (foam compression): 70% base + impact severity
2. **Tangential velocity modification** via spin-surface friction coupling
3. **Friction torque** alters spin direction

### Bounce-Out Risk Scoring

The solver evaluates bounce-out probability using:

```java
bounceRisk = verticalVelocityRisk × 0.6 + entryAngleRisk × 0.4 + spinAlignment
```

**Spin Alignment Bonus/Penalty:**
- **Good backspin** (aligned with entry): -20% risk
- **Forward spin**: +30% risk
- **Neutral spin**: No adjustment

### Favorable Spin Conditions

For **minimum bounce-out**:
- Backspin (Y-axis, positive)
- Spin magnitude > 100 rad/s
- Downward entry velocity > 1 m/s
- Entry angle > 30°

---

## Calibration & Tuning

### Spin-Related Parameters

```java
CalibrationParameters params = new CalibrationParameters()
    .withMagnusCoefficient(0.00002)     // Increase for stronger Magnus effect
    .withSpinEfficiency(0.92)            // Shooter wheel spin transfer
    .withSpinDecayRate(0.06)             // Foam ball spin loss rate
    .withSpinCorrectionFactor(1.05);     // Per-shot fine-tuning
```

### AdvantageKit Logging

All spin parameters are automatically logged:

```
Shooter/Trajectory/Input/SpinRate
Shooter/Trajectory/Input/SpinAxisX
Shooter/Trajectory/Input/SpinAxisY
Shooter/Trajectory/Input/SpinAxisZ
Shooter/Trajectory/Calibration/SpinEfficiency
Shooter/Trajectory/Calibration/SpinDecayRate
```

### Incremental Calibration

Enable automatic tuning from match data:

```java
controller.setIncrementalCalibrationEnabled(true);
controller.setLearningRate(0.05);  // 5% adjustment per shot

// System automatically adjusts spin efficiency based on hit rate
// with high-spin shots
```

---

## Safety Limits

| Parameter | Min | Max | Default |
|-----------|-----|-----|---------|
| Spin Rate | 0.0 | 500.0 rad/s | 200.0 rad/s |
| Magnus Coefficient | 0.0 | 0.001 | 0.00001 |
| Spin Efficiency | 0.5 | 1.0 | 0.90 |
| Spin Decay Rate | 0.0 | 0.5 /s | 0.05 /s |

Exceeding limits triggers automatic clamping with warning logs.

---

## Best Practices

### For Consistent Hits

1. **Use moderate backspin** (150-250 rad/s)
2. **Avoid excessive spin** (> 400 rad/s) unless necessary
3. **Calibrate spin efficiency** from logged match data
4. **Monitor spin decay** in AdvantageScope during matches

### For Long-Range Shots

1. **Increase backspin** (300-400 rad/s) to extend range
2. **Ensure downward entry** to minimize bounce-out
3. **Test Magnus coefficient** tuning for your robot's ball condition

### For Near-Rim Shots

1. **Reduce spin** (100-150 rad/s) to minimize risk
2. **Favor steep entry angles** (> 45°)
3. **Avoid sidespin** which can cause rim contact

---

## Example: Competition-Ready Configuration

```java
public class ShooterSubsystem {
    private final AdvantageKitTrajectoryHelper trajectory;
    
    public ShooterSubsystem() {
        // Start with calibrated parameters
        CalibrationParameters params = new CalibrationParameters()
            .withMagnusCoefficient(0.000015)  // Tuned from practice
            .withSpinEfficiency(0.93)          // Measured from shooter
            .withSpinDecayRate(0.055);         // Foam ball specific
        
        trajectory = new AdvantageKitTrajectoryHelper("Shooter/Trajectory", params);
        
        // Enable logging during practice
        trajectory.setLoggingEnabled(true);
    }
    
    public void aimAt(Pose2d robotPose) {
        // Calculate with optimized backspin
        double optimalSpin = calculateOptimalSpin(robotPose);
        
        var solution = trajectory.calculateAndLog(
            robotPose.getX(), robotPose.getY(),
            SHOOTER_HEIGHT, LAUNCH_SPEED, optimalSpin
        );
        
        setAngles(solution.launchYawDeg, solution.launchPitchDeg);
    }
    
    private double calculateOptimalSpin(Pose2d pose) {
        double distance = pose.getTranslation().getNorm();
        
        // Long range: more spin for lift
        if (distance > 6.0) return 300.0;
        
        // Medium range: moderate spin
        if (distance > 4.0) return 200.0;
        
        // Short range: minimal spin to reduce bounce-out
        return 120.0;
    }
}
```

---

## Testing & Validation

### Run Spin Tests

```bash
mvn test -Dtest=SpinPhysicsTest
```

### Key Test Cases

- Spin rate variations (0-500 rad/s)
- Spin axis deviations (all directions)
- Magnus force verification
- Spin decay over time
- Rim bounce with spin
- Deterministic behavior

All tests must complete in <100ms per shot for real-time compliance.

---

## Troubleshooting

### Problem: Magnus effect seems too weak

**Solution**: Increase Magnus coefficient
```java
params.withMagnusCoefficient(0.00003);  // 3x default
```

### Problem: Ball spins too fast/slow compared to commanded

**Solution**: Adjust spin efficiency
```java
params.withSpinEfficiency(0.88);  // Reduce if overspin
```

### Problem: Shots bounce out despite good entry

**Solution**: Check spin alignment - may have forward spin
```java
// Verify backspin (Y > 0)
trajectory.calculateAndLog(x, y, z, speed, spinRate, 0, 1, 0);
```

### Problem: Spin decays too quickly

**Solution**: Lower spin decay rate
```java
params.withSpinDecayRate(0.03);  // Slower decay
```

---

## References

- **Physics Model**: `PhysicsModel.java` - `computeDerivative()` and `applyCollision()`
- **Bounce Risk**: `InverseSolver.java` - `calculateBounceOutRisk()`
- **Test Suite**: `SpinPhysicsTest.java` - 27 comprehensive tests
- **AdvantageKit**: `AdvantageKitTrajectoryHelper.java` - Spin logging

For questions or issues, see the main README or contact the TrajectoryCalc maintainers.
