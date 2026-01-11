package frc.robot.trajectory.simulation;

import java.util.ArrayList;
import java.util.List;
import frc.robot.trajectory.calibration.CalibrationParameters;
import frc.robot.trajectory.physics.HubGeometry;
import frc.robot.trajectory.physics.PhysicsConstants;
import frc.robot.trajectory.physics.PhysicsModel;
import frc.robot.trajectory.physics.ProjectileState;
import frc.robot.trajectory.solver.RK4Integrator;
import frc.robot.trajectory.util.Vector3D;

import java.util.ArrayList;
import java.util.List;

/**
 * Forward trajectory simulator with full physics including drag, Magnus effect, and collisions.
 */
public class TrajectorySimulator {
    private final PhysicsModel physicsModel;
    private final RK4Integrator integrator;
    private final HubGeometry hubGeometry;
    private final double maxSimulationTime;
    
    public TrajectorySimulator() {
        this(new PhysicsModel(), new HubGeometry());
    }
    
    public TrajectorySimulator(PhysicsModel physicsModel, HubGeometry hubGeometry) {
        this.physicsModel = physicsModel;
        this.integrator = new RK4Integrator(physicsModel);
        this.hubGeometry = hubGeometry;
        this.maxSimulationTime = PhysicsConstants.MAX_SIMULATION_TIME;
    }
    
    /**
     * Simulate the full trajectory from initial state until it hits the ground or times out.
     * Returns the trajectory as a list of states.
     */
    public TrajectoryResult simulate(ProjectileState initialState) {
        List<ProjectileState> trajectory = new ArrayList<>();
        ProjectileState state = initialState;
        trajectory.add(state);
        
        boolean hitTarget = false;
        boolean crossedOpeningPlane = false;
        ProjectileState entryState = null;
        double entryScore = 0.0;
        
        while (state.time < maxSimulationTime) {
            // Check if we've hit the ground
            if (state.position.z < 0) {
                break;
            }
            
            // Check if we're crossing the opening plane
            if (!crossedOpeningPlane && hubGeometry.isAtOpeningPlane(state, 0.05)) {
                crossedOpeningPlane = true;
                entryState = state;
                
                // Check if it's a valid hit
                if (hubGeometry.hasEnoughClearance(state.position, 1.2)) {
                    hitTarget = true;
                    entryScore = hubGeometry.evaluateEntry(state);
                }
            }
            
            // Integrate one step
            state = integrator.step(state);
            trajectory.add(state);
            
            // Early exit if we've passed well below the target
            if (state.position.z < -1.0) {
                break;
            }
        }
        
        return new TrajectoryResult(trajectory, hitTarget, entryState, entryScore);
    }
    
    /**
     * Simulate with shooter efficiency applied to initial conditions and full spin vector control.
     */
    public TrajectoryResult simulateWithShooterModel(Vector3D robotPosition,
                                                      double launchSpeed,
                                                      double launchYawDeg,
                                                      double launchPitchDeg,
                                                      Vector3D spin) {
        CalibrationParameters params = physicsModel.getParameters();
        
        // Apply shooter efficiency
        double actualSpeed = launchSpeed * params.speedEfficiency * params.speedCorrectionFactor;
        Vector3D actualSpin = spin.scale(params.spinEfficiency * params.spinCorrectionFactor);
        
        // Convert angles to radians
        double yaw = Math.toRadians(launchYawDeg);
        double pitch = Math.toRadians(launchPitchDeg);
        
        // Compute initial velocity vector
        double vx = actualSpeed * Math.cos(pitch) * Math.cos(yaw);
        double vy = actualSpeed * Math.cos(pitch) * Math.sin(yaw);
        double vz = actualSpeed * Math.sin(pitch);
        Vector3D velocity = new Vector3D(vx, vy, vz);
        
        ProjectileState initialState = new ProjectileState(robotPosition, velocity, actualSpin, 0.0);
        return simulate(initialState);
    }
    
    /**
     * Simulate with shooter efficiency (backward compatibility - assumes backspin).
     */
    public TrajectoryResult simulateWithShooterModel(Vector3D robotPosition,
                                                      double launchSpeed,
                                                      double launchYawDeg,
                                                      double launchPitchDeg,
                                                      double spinRate) {
        // Default: backspin around Y-axis
        Vector3D spin = new Vector3D(0, spinRate, 0);
        return simulateWithShooterModel(robotPosition, launchSpeed, launchYawDeg, launchPitchDeg, spin);
    }
    
    public HubGeometry getHubGeometry() {
        return hubGeometry;
    }
    
    /**
     * Result of a trajectory simulation.
     */
    public static class TrajectoryResult {
        public final List<ProjectileState> trajectory;
        public final boolean hitTarget;
        public final ProjectileState entryState;
        public final double entryScore;
        
        public TrajectoryResult(List<ProjectileState> trajectory, boolean hitTarget,
                               ProjectileState entryState, double entryScore) {
            this.trajectory = trajectory;
            this.hitTarget = hitTarget;
            this.entryState = entryState;
            this.entryScore = entryScore;
        }
        
        public double getMaxHeight() {
            return trajectory.stream()
                .mapToDouble(s -> s.position.z)
                .max()
                .orElse(0.0);
        }
        
        public double getFlightTime() {
            if (trajectory.isEmpty()) return 0.0;
            return trajectory.get(trajectory.size() - 1).time;
        }
    }
}
