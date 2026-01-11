# AdvantageKit Tuning Guide

This guide shows how to use AdvantageKit logs from your robot to tune the trajectory calculator parameters.

## Overview

The trajectory calculator needs to be tuned to match your specific robot's characteristics:
- Ball compression in your shooter
- Wheel wear and grip
- Actual launch velocity vs commanded
- Air resistance (can vary with ball wear)

Your AdvantageKit logs will provide real-world data to tune these parameters automatically.

## Quick Setup

### 1. Install the Library

Add `TrajectoryCalc.json` to your `vendordeps/` folder using WPILib's vendor library manager.

### 2. Add to Your Shooter Subsystem

```java
import com.ruthiefrc.trajectory.*;
import com.ruthiefrc.trajectory.advantagekit.TrajectoryLogger;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterController trajectoryCalc;
    
    public ShooterSubsystem() {
        trajectoryCalc = new ShooterController();
    }
    
    @Override
    public void periodic() {
        // Your existing code...
        
        // Log trajectory data every loop
        logTrajectoryData();
    }
    
    private void logTrajectoryData() {
        // Log current calibration parameters
        CalibrationParameters params = trajectoryCalc.getCalibrationParameters();
        Logger.recordOutput("Trajectory/Calibration/DragCoeff", params.dragCoefficient);
        Logger.recordOutput("Trajectory/Calibration/SpeedEfficiency", params.speedEfficiency);
        Logger.recordOutput("Trajectory/Calibration/SpinEfficiency", params.spinEfficiency);
        Logger.recordOutput("Trajectory/Calibration/MagnusCoeff", params.magnusCoefficient);
    }
}
```

### 3. Log Each Shot

```java
public void shoot() {
    // Get current robot position from your drivetrain
    Pose2d currentPose = drivetrain.getPose();
    
    // Calculate trajectory
    InverseSolver.SolutionResult solution = trajectoryCalc.calculateShot(
        currentPose.getX(),
        currentPose.getY(), 
        SHOOTER_HEIGHT,
        commandedSpeed
    );
    
    // Log shot INPUTS
    Logger.recordOutput("Trajectory/Shot/Input/RobotX", currentPose.getX());
    Logger.recordOutput("Trajectory/Shot/Input/RobotY", currentPose.getY());
    Logger.recordOutput("Trajectory/Shot/Input/CommandedSpeed", commandedSpeed);
    Logger.recordOutput("Trajectory/Shot/Input/Timestamp", System.currentTimeMillis());
    
    // Log calculated trajectory
    Logger.recordOutput("Trajectory/Shot/Output/Yaw", solution.launchYawDeg);
    Logger.recordOutput("Trajectory/Shot/Output/Pitch", solution.launchPitchDeg);
    Logger.recordOutput("Trajectory/Shot/Output/PredictedHit", solution.isHit());
    Logger.recordOutput("Trajectory/Shot/Output/Score", solution.score);
    
    // Actually shoot
    setTurretAngle(solution.launchYawDeg);
    setHoodAngle(solution.launchPitchDeg);
    setFlywheelSpeed(commandedSpeed);
    feed();
    
    // After shot lands (1-2 seconds later), log the RESULT
    // This can be from vision, game announcer, or manual observation
    Timer.delay(1.5);
    boolean actualHit = checkIfScored(); // Your hit detection
    
    Logger.recordOutput("Trajectory/Shot/Result/ActualHit", actualHit);
    Logger.recordOutput("Trajectory/Shot/Result/Timestamp", System.currentTimeMillis());
    
    // Log to calibration system
    trajectoryCalc.logShot(
        currentPose.getX(), currentPose.getY(), SHOOTER_HEIGHT,
        solution, commandedSpeed, SPIN_RATE, actualHit
    );
}
```

## Analyzing AdvantageKit Logs

### Step 1: Review Logs in AdvantageScope

1. Open your match/practice log in AdvantageScope
2. Look for the `Trajectory/` namespace
3. You should see:
   - `Trajectory/Shot/Input/*` - Robot position, commanded speed for each shot
   - `Trajectory/Shot/Output/*` - Calculated angles and predictions
   - `Trajectory/Shot/Result/*` - Whether shot actually scored
   - `Trajectory/Calibration/*` - Current tuning parameters

### Step 2: Identify Patterns

Look for systematic errors:

**Pattern 1: All shots falling short**
- Likely: Speed efficiency too high OR drag coefficient too high
- Solution: Decrease `speedEfficiency` or decrease `dragCoefficient`

**Pattern 2: All shots going long**
- Likely: Speed efficiency too low OR drag coefficient too low  
- Solution: Increase `speedEfficiency` or increase `dragCoefficient`

**Pattern 3: Close shots accurate, far shots inaccurate**
- Likely: Drag coefficient incorrect
- Solution: Adjust `dragCoefficient` based on range

**Pattern 4: Shots drift left/right**
- Likely: Magnus effect from spin OR turret calibration
- Solution: Adjust `magnusCoefficient` or check turret angle calibration

### Step 3: Calculate Corrections

#### Speed Efficiency

From your logs, find average flywheel RPM vs commanded:

```python
# In Python (or use AdvantageScope's calculated fields)
commanded_speed_mps = 12.0  # Your target
actual_rpm = 3500  # From motor controller logs

wheel_circumference = 0.1  # meters (your wheel)
actual_speed_mps = (actual_rpm * wheel_circumference) / 60

speed_efficiency = actual_speed_mps / commanded_speed_mps
# e.g., 11.2 / 12.0 = 0.933
```

Update in your code:
```java
CalibrationParameters params = new CalibrationParameters()
    .withSpeedEfficiency(0.933);
trajectoryCalc = new ShooterController(params);
```

#### Drag Coefficient

If you have systematic overshoots/undershoots:

1. Count your shots from 3-5m range
2. If 80% fall short: decrease drag by 5% (e.g., 0.47 → 0.45)
3. If 80% go long: increase drag by 5% (e.g., 0.47 → 0.49)
4. Test and iterate

```java
CalibrationParameters params = new CalibrationParameters()
    .withDragCoefficient(0.45);  // Adjusted from 0.47
```

## Automated Tuning from Logs

### Batch Calibration (Traditional)

The library can auto-tune from your logged shots:

```java
// After a practice session, load all shots
public void calibrateFromPractice() {
    // Shots are already logged via trajectoryCalc.logShot()
    
    // Run calibration
    boolean updated = trajectoryCalc.calibrate();
    
    if (updated) {
        CalibrationParameters newParams = trajectoryCalc.getCalibrationParameters();
        System.out.println("New parameters: " + newParams);
        
        // Save to file or NetworkTables for next match
        saveParameters(newParams);
    }
    
    // View stats
    System.out.println(trajectoryCalc.getCalibrationStats());
}
```

### Incremental Calibration (NEW - Real-Time Learning)

Enable automatic parameter updates after each shot during practice:

```java
// In your subsystem constructor or init
public void initPracticeMode() {
    // Enable incremental calibration for practice
    trajectoryCalc.setIncrementalCalibrationEnabled(true);
    
    // Set learning rate (0.0-1.0, default: 0.05)
    trajectoryCalc.setLearningRate(0.05); // 5% adjustment per shot
    
    // Parameters now update automatically after each logged shot!
}

public void initCompetitionMode() {
    // Disable for stable competition performance
    trajectoryCalc.setIncrementalCalibrationEnabled(false);
}
```

**How Incremental Calibration Works:**
1. Logs each shot result (hit/miss)
2. Analyzes recent performance window (last 5 shots)
3. If hit rate < 60%, adjusts drag and speed efficiency
4. Updates parameters in real-time without manual intervention
5. Requires 5 baseline shots before starting adjustments

**When to Use:**
- ✅ **Practice sessions** - Continuous improvement
- ✅ **Testing new balls** - Fast adaptation
- ✅ **Pre-match warmup** - Quick tuning
- ❌ **Competition matches** - Use stable parameters

**Monitoring Incremental Updates:**

```java
// Log parameter changes in AdvantageKit
@Override
public void periodic() {
    CalibrationParameters params = trajectoryCalc.getCalibrationParameters();
    Logger.recordOutput("Trajectory/Calibration/DragCoeff", params.dragCoefficient);
    Logger.recordOutput("Trajectory/Calibration/SpeedEfficiency", params.speedEfficiency);
    Logger.recordOutput("Trajectory/Calibration/IncrementalEnabled", 
        trajectoryCalc.isIncrementalCalibrationEnabled());
}
```

**Best Practices:**
- Start with learning rate of 0.05 (5%)
- Increase to 0.1 (10%) for faster adaptation if needed
- Monitor in AdvantageScope to see parameter evolution
- Let it run for 15-20 shots to stabilize
- Save final parameters for competition use

### Pre-Seeded Guesses (NEW - Faster Convergence)

The solver automatically caches successful shots for faster convergence:

```java
// No configuration needed - enabled by default!
// First shot from a position uses geometric estimate
// Subsequent shots from nearby positions use cached angles

// Shots are cached on a 0.5m position grid
// Each successful shot improves future solves from that area
```

**Benefits:**
- 10-30% faster solve times for repeated positions
- More consistent angle solutions
- Automatic - no manual management needed
- Cache clears automatically if disabled

**Viewing Cache Performance:**

```java
// See solve times in logs
long startTime = System.nanoTime();
var solution = trajectoryCalc.calculateShot(x, y, z, speed, spin);
long duration = (System.nanoTime() - startTime) / 1_000_000;

Logger.recordOutput("Trajectory/SolveTime", duration);
```

Expect:
- First solve from new position: 40-70ms
- Cached solve: 20-50ms (faster due to better initial guess)


## Logging Best Practices

### During Practice

```java
// Log EVERYTHING during practice
Logger.recordOutput("Trajectory/Shot/Input/BallCondition", "new"); // or "worn"
Logger.recordOutput("Trajectory/Shot/Input/Distance", getDistanceToTarget());
Logger.recordOutput("Trajectory/Shot/Input/Angle", getAngleToTarget());

// Log shooter state
Logger.recordOutput("Shooter/Flywheel/CommandedRPM", commandedRPM);
Logger.recordOutput("Shooter/Flywheel/ActualRPM", getActualRPM());
Logger.recordOutput("Shooter/Flywheel/Current", getMotorCurrent());
```

### During Matches

```java
// Log only essentials during matches (reduce CPU load)
Logger.recordOutput("Trajectory/Shot/Input/RobotX", x);
Logger.recordOutput("Trajectory/Shot/Input/RobotY", y);
Logger.recordOutput("Trajectory/Shot/Output/Yaw", yaw);
Logger.recordOutput("Trajectory/Shot/Output/Pitch", pitch);
Logger.recordOutput("Trajectory/Shot/Result/Hit", hit);
```

## Example Tuning Session

### Session 1: Initial Baseline

1. Start with defaults:
```java
CalibrationParameters params = new CalibrationParameters(); // All defaults
```

2. Take 20 shots from various positions
3. Log in AdvantageScope: 5/20 hit = 25% accuracy
4. Note in AdvantageScope: All 15 misses were SHORT

### Session 2: Adjust Speed Efficiency

1. Measure actual flywheel speed: 95% of commanded
2. Update:
```java
params = params.withSpeedEfficiency(0.95);
```

3. Take 20 more shots
4. Log shows: 12/20 hit = 60% accuracy
5. Note: Closer range shots still slightly short

### Session 3: Adjust Drag

1. Decrease drag coefficient:
```java
params = params.withDragCoefficient(0.45); // from 0.47
```

2. Take 20 more shots  
3. Log shows: 16/20 hit = 80% accuracy
4. Note: Good accuracy across all ranges!

### Session 4: Fine-tuning

1. Let auto-calibration run:
```java
trajectoryCalc.calibrate();
```

2. Take 20 final shots
3. Log shows: 18/20 hit = 90% accuracy
4. Save parameters for competition!

## Advanced: Custom Hit Detection

If you have vision or other sensors:

```java
public class VisionHitDetector {
    private final PhotonCamera camera;
    
    public boolean checkIfScored() {
        // Wait for ball to enter goal
        Timer.delay(1.0);
        
        // Check if we see the ball in the goal
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            // Ball is visible in goal area
            return true;
        }
        
        // Or use game data from FMS
        return DriverStation.getGameSpecificMessage().contains("SCORE");
    }
}
```

## Troubleshooting

### "Logs show 0 shots"
- Verify `Logger.recordOutput()` calls are in your code
- Check AdvantageScope is connected
- Ensure you're calling `shoot()` method

### "Calibration doesn't improve accuracy"
- Need at least 10 shots for calibration
- Check if you're saving/loading parameters between runs
- Verify hit detection is accurate

### "Parameters keep changing"
- Normal during initial tuning
- After 50+ shots, parameters should stabilize
- Lock parameters for competition:
```java
// Don't call trajectoryCalc.calibrate() during competition
```

## Integration Checklist

- [ ] Install TrajectoryCalc vendordep
- [ ] Add ShooterController to your shooter subsystem
- [ ] Log shot inputs in `shoot()` method
- [ ] Log shot results after each shot
- [ ] Log calibration parameters in `periodic()`
- [ ] Test with 10+ shots and verify logs in AdvantageScope
- [ ] Run initial calibration from logs
- [ ] Tune parameters manually if needed
- [ ] Test tuned parameters in practice
- [ ] Lock parameters for competition

## Support

Having issues with AdvantageKit integration?
- Check AdvantageScope shows your logs under `Trajectory/`
- Verify Logger.recordOutput() calls are being executed
- Test with standalone logging first (without trajectory calc)
- Post logs in GitHub Issues for help with tuning

## See Also

- [WPILIB_INTEGRATION.md](WPILIB_INTEGRATION.md) - Full WPILib integration guide
- [INTEGRATION.md](INTEGRATION.md) - General usage and API reference
- [examples/wpilib/](examples/wpilib/) - Complete example code
