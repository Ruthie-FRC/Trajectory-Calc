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
        boolean clearedRimVertically = false;
        ProjectileState entryState = null;
        double entryScore = 0.0;
        
        // Rim height validation - need to clear rim with ball radius margin
        double rimHeight = hubGeometry.getOpeningHeight();
        double ballRadius = PhysicsConstants.DEFAULT_BALL_RADIUS;
        double minClearanceHeight = rimHeight + ballRadius + 0.05; // 5cm extra margin for safety
        
        while (state.time < maxSimulationTime) {
            // Check if we've hit the ground
            if (state.position.z < 0) {
                break;
            }
            
            // Check if trajectory passes over the target area (XY plane)
            double dx = state.position.x - hubGeometry.getCenter().x;
            double dy = state.position.y - hubGeometry.getCenter().y;
            double distFromCenter = Math.sqrt(dx * dx + dy * dy);
            double targetRadius = hubGeometry.getOpeningRadius();
            
            // When ball is over the target horizontally, check vertical clearance
            if (distFromCenter <= targetRadius && !clearedRimVertically) {
                if (state.position.z >= minClearanceHeight) {
                    clearedRimVertically = true;
                }
            }
            
            // Check if we're crossing the opening plane (descending through target height)
            if (!crossedOpeningPlane && hubGeometry.isAtOpeningPlane(state, 0.05) && state.velocity.z < 0) {
                crossedOpeningPlane = true;
                entryState = state;
                
                // Check if it's a valid hit - must have cleared rim AND be entering center area
                // Aim for center 60% of opening, not just any part
                double centerRadius = targetRadius * 0.6; // Target center 60% of opening
                
                // CRITICAL: Check for bounce-off risk
                // Calculate entry angle relative to vertical
                double speed = state.velocity.magnitude();
                double entryAngle = 0;
                if (speed > 0.1) {
                    // Entry angle: 0° = straight down, 90° = horizontal
                    entryAngle = Math.abs(Math.toDegrees(Math.asin(-state.velocity.z / speed)));
                }
                
                // Prevent bounce-off: entry angle must be steep enough (< 60° from vertical)
                // and downward velocity must be sufficient
                double minDownwardVelocity = 2.0; // m/s minimum downward component
                boolean safeEntryAngle = entryAngle < 60.0;
                boolean sufficientDownwardVel = state.velocity.z < -minDownwardVelocity;
                
                if (clearedRimVertically && 
                    distFromCenter <= centerRadius && 
                    safeEntryAngle && 
                    sufficientDownwardVel) {
                    hitTarget = true;
                    entryScore = hubGeometry.evaluateEntry(state);
                    
                    // Penalize if too close to rim edge (bounce risk)
                    double rimProximityFactor = distFromCenter / centerRadius;
                    if (rimProximityFactor > 0.7) {
                        entryScore *= 0.8; // 20% penalty for edge proximity
                    }
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
