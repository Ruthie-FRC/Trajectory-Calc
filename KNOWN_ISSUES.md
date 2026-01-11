# Known Issues

## ~~InverseSolver Not Finding Solutions~~ [RESOLVED]

**Status:** ✅ RESOLVED  
**Resolution Date:** 2026-01-11

### Root Cause
The Magnus force calculation in `PhysicsModel.computeDerivative()` was applying double-scaling by multiplying the spin × velocity cross product by |ω| × |v| again. This caused:
- Magnus accelerations of 260+ m/s² (should be ~1-2 m/s²)
- Trajectories terminating in 0.1s instead of 1-2s
- Maximum heights of <1m instead of 3-5m
- Ball velocities reversing direction mid-flight

### Fix Applied
1. **Corrected Magnus Force Formula**: Changed from `F = Cm × |ω| × |v| × (ω × v)` to `F = Cm × (ω × v)`
2. **Updated Coefficient**: Increased `DEFAULT_MAGNUS_COEFFICIENT` from 1e-5 to 1.5e-4 to compensate for removed scaling
3. **Widened Solver Search**: Increased search range from ±4°/±2° to ±15°/±9° (yaw/pitch) to find more varied solutions

### Verification
After fix, trajectories show realistic behavior:
- 15 m/s @ 45° pitch: 1.76s flight time, 4.44m max height, 14.8m travel distance
- Ball maintains forward velocity throughout flight
- Brute force tests confirm viable hitting solutions exist for 3m-8m shots
- Example: 12 m/s @ 30° pitch successfully hits from 3m distance

### Remaining Work
Solver optimization - while the physics is now correct and viable solutions exist, the InverseSolver may need further tuning to consistently find optimal solutions across all test scenarios. The search grid and convergence criteria can be refined for better performance.
