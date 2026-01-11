# Known Issues

## InverseSolver Not Finding Solutions

**Status:** Under Investigation  
**Severity:** High  
**Affects:** All trajectory calculations

### Description
The `InverseSolver` is currently not finding viable shooting solutions for any test scenarios. All shots return "NO VIABLE SHOT FOUND" with score values around -1.0.

### Symptoms
- All trajectory simulations result in very short flight times (0.1-0.2 seconds)
- Maximum heights are unrealistically low (< 1 meter)
- The solver appears to be using only geometric estimates without refining them
- Ball trajectories suggest the simulation is terminating prematurely

### Investigation Notes
Testing with:
- Robot at (-4, 0, 0.5)m
- Target at (0, 0, 1.83)m (HUB center)
- Speed: 15 m/s
- Pitch angles: 20-60°

All tests show:
- Flight times < 0.15s
- Max heights < 1m
- No successful target hits

Expected behavior for 15 m/s at 60° pitch:
- Flight time: ~1-2 seconds
- Max height: ~5-8 meters
- Should easily reach 4m horizontal distance

### Possible Causes
1. Issue with RK4 integrator timestep
2. Problem with simulation termination conditions  
3. Physics model misconfiguration
4. Coordinate system mismatch

### Impact on This PR
This issue appears to be **pre-existing** in the codebase. The changes made in this PR (RPM constraint, file format support, single-command execution) are independent of the solver's ability to find solutions.

### Next Steps
1. Debug the trajectory simulator to understand why simulations terminate early
2. Verify the physics model parameters are reasonable
3. Check if there's a coordinate system issue (field coordinates vs robot-relative)
4. Add unit tests for the simulator with known-good trajectories
5. Consider adding diagnostic logging to trace simulation steps

### Workaround
None currently available. The solver needs to be fixed before trajectory calculations will work correctly.
