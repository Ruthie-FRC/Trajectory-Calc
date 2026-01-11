package com.ruthiefrc.trajectory;

/**
 * Inverse ballistic solver using iterative shooting method.
 * Solves for launch angles given robot pose, target, and launch speed.
 */
public class InverseSolver {
    private final TrajectorySimulator simulator;
    private final HubGeometry hubGeometry;
    private final int maxIterations;
    private final double convergenceTolerance;
    
    public InverseSolver(TrajectorySimulator simulator) {
        this(simulator, 50, PhysicsConstants.CONVERGENCE_TOLERANCE);
    }
    
    public InverseSolver(TrajectorySimulator simulator, int maxIterations, double convergenceTolerance) {
        this.simulator = simulator;
        this.hubGeometry = simulator.getHubGeometry();
        this.maxIterations = maxIterations;
        this.convergenceTolerance = convergenceTolerance;
    }
    
    /**
     * Solve for launch parameters to hit the target.
     * Returns the optimal launch angles and expected hit quality.
     */
    public SolutionResult solve(Vector3D robotPosition, double nominalLaunchSpeed, double spinRate) {
        Vector3D targetCenter = hubGeometry.getCenter();
        
        // Initial guess for launch angles based on geometric aim
        double dx = targetCenter.x - robotPosition.x;
        double dy = targetCenter.y - robotPosition.y;
        double dz = targetCenter.z - robotPosition.z;
        double horizontalDist = Math.sqrt(dx * dx + dy * dy);
        
        // Initial yaw angle (direction to target)
        double yawDeg = Math.toDegrees(Math.atan2(dy, dx));
        
        // Initial pitch angle (rough ballistic estimate)
        double pitchDeg = Math.toDegrees(Math.atan2(dz, horizontalDist)) + 5.0; // add elevation
        
        // Iterative refinement using gradient descent / shooting method
        double bestYaw = yawDeg;
        double bestPitch = pitchDeg;
        double bestScore = -1.0;
        TrajectorySimulator.TrajectoryResult bestResult = null;
        
        // Grid search around initial guess
        double yawStep = 2.0; // degrees
        double pitchStep = 1.0; // degrees
        
        for (int iter = 0; iter < 3; iter++) {
            double currentBestScore = bestScore;
            
            for (double yawOffset = -yawStep * 2; yawOffset <= yawStep * 2; yawOffset += yawStep) {
                for (double pitchOffset = -pitchStep * 2; pitchOffset <= pitchStep * 2; pitchOffset += pitchStep) {
                    double tryYaw = bestYaw + yawOffset;
                    double tryPitch = bestPitch + pitchOffset;
                    
                    // Simulate trajectory
                    TrajectorySimulator.TrajectoryResult result = simulator.simulateWithShooterModel(
                        robotPosition, nominalLaunchSpeed, tryYaw, tryPitch, spinRate
                    );
                    
                    // Evaluate result
                    double score = evaluateSolution(result, targetCenter);
                    
                    if (score > bestScore) {
                        bestScore = score;
                        bestYaw = tryYaw;
                        bestPitch = tryPitch;
                        bestResult = result;
                    }
                }
            }
            
            // Refine search around best solution
            yawStep *= 0.5;
            pitchStep *= 0.5;
            
            // Check for convergence
            if (Math.abs(bestScore - currentBestScore) < 0.001) {
                break;
            }
        }
        
        // Fine-tuning phase with smaller steps
        yawStep = 0.2;
        pitchStep = 0.1;
        
        for (int iter = 0; iter < maxIterations; iter++) {
            boolean improved = false;
            
            // Try small adjustments
            double[][] deltas = {{yawStep, 0}, {-yawStep, 0}, {0, pitchStep}, {0, -pitchStep}};
            
            for (double[] delta : deltas) {
                double tryYaw = bestYaw + delta[0];
                double tryPitch = bestPitch + delta[1];
                
                TrajectorySimulator.TrajectoryResult result = simulator.simulateWithShooterModel(
                    robotPosition, nominalLaunchSpeed, tryYaw, tryPitch, spinRate
                );
                
                double score = evaluateSolution(result, targetCenter);
                
                if (score > bestScore) {
                    bestScore = score;
                    bestYaw = tryYaw;
                    bestPitch = tryPitch;
                    bestResult = result;
                    improved = true;
                }
            }
            
            if (!improved) {
                // Reduce step size and continue
                yawStep *= 0.5;
                pitchStep *= 0.5;
                
                if (yawStep < 0.01 && pitchStep < 0.01) {
                    break; // Converged
                }
            }
        }
        
        return new SolutionResult(bestYaw, bestPitch, bestScore, bestResult);
    }
    
    /**
     * Evaluate the quality of a solution.
     * Higher score is better.
     */
    private double evaluateSolution(TrajectorySimulator.TrajectoryResult result, Vector3D targetCenter) {
        if (!result.hitTarget) {
            // Miss penalty - return negative score based on miss distance
            if (result.entryState != null) {
                double dx = result.entryState.position.x - targetCenter.x;
                double dy = result.entryState.position.y - targetCenter.y;
                double missDistance = Math.sqrt(dx * dx + dy * dy);
                return -missDistance;
            }
            return -100.0; // Large penalty for no entry state
        }
        
        // Hit - return entry score
        return result.entryScore;
    }
    
    /**
     * Result of the inverse solver.
     */
    public static class SolutionResult {
        public final double launchYawDeg;
        public final double launchPitchDeg;
        public final double score;
        public final TrajectorySimulator.TrajectoryResult trajectory;
        
        public SolutionResult(double launchYawDeg, double launchPitchDeg, 
                             double score, TrajectorySimulator.TrajectoryResult trajectory) {
            this.launchYawDeg = launchYawDeg;
            this.launchPitchDeg = launchPitchDeg;
            this.score = score;
            this.trajectory = trajectory;
        }
        
        public boolean isHit() {
            return trajectory != null && trajectory.hitTarget;
        }
        
        @Override
        public String toString() {
            return String.format("Solution[yaw=%.2f°, pitch=%.2f°, score=%.3f, hit=%s]",
                launchYawDeg, launchPitchDeg, score, isHit());
        }
    }
}
