package com.ruthiefrc.trajectory.advantagekit;

import com.ruthiefrc.trajectory.*;

/**
 * AdvantageKit-compatible logging interface for trajectory calculations.
 * Use with AdvantageKit's Logger.recordOutput() for automatic logging.
 * 
 * Example usage:
 * <pre>
 * {@code
 * // In your subsystem's periodic():
 * TrajectoryLogger.Inputs inputs = TrajectoryLogger.createInputs(...);
 * TrajectoryLogger.Outputs outputs = TrajectoryLogger.toOutputs(solution, calcTime);
 * 
 * // Log with AdvantageKit:
 * Logger.recordOutput("Trajectory/Input/RobotX", inputs.robotX);
 * Logger.recordOutput("Trajectory/Output/LaunchYaw", outputs.launchYawDeg);
 * }
 * </pre>
 */
public class TrajectoryLogger {
    
    /**
     * Loggable inputs for trajectory calculation.
     * Compatible with AdvantageKit's logging system.
     */
    public static class Inputs {
        public double robotX = 0.0;
        public double robotY = 0.0;
        public double robotZ = 0.0;
        public double targetX = 0.0;
        public double targetY = 0.0;
        public double targetZ = 0.0;
        public double launchSpeed = 0.0;
        public double spinRate = 0.0;
        public long timestamp = 0;
    }
    
    /**
     * Loggable outputs from trajectory calculation.
     * Compatible with AdvantageKit's logging system.
     */
    public static class Outputs {
        public double launchYawDeg = 0.0;
        public double launchPitchDeg = 0.0;
        public double solutionScore = 0.0;
        public boolean isHit = false;
        public double flightTime = 0.0;
        public double maxHeight = 0.0;
        public int trajectoryPoints = 0;
        public long calculationTimeMs = 0;
    }
    
    /**
     * Loggable calibration parameters.
     * Compatible with AdvantageKit's logging system.
     */
    public static class Calibration {
        public double dragCoefficient = 0.0;
        public double magnusCoefficient = 0.0;
        public double speedEfficiency = 0.0;
        public double spinEfficiency = 0.0;
        public double restitutionCoefficient = 0.0;
        public double frictionCoefficient = 0.0;
        public int shotCount = 0;
        public double hitRate = 0.0;
    }
    
    /**
     * Convert InverseSolver.SolutionResult to loggable outputs.
     */
    public static Outputs toOutputs(InverseSolver.SolutionResult solution, long calculationTimeMs) {
        Outputs outputs = new Outputs();
        if (solution != null) {
            outputs.launchYawDeg = solution.launchYawDeg;
            outputs.launchPitchDeg = solution.launchPitchDeg;
            outputs.solutionScore = solution.score;
            outputs.isHit = solution.isHit();
            
            if (solution.trajectory != null) {
                outputs.flightTime = solution.trajectory.getFlightTime();
                outputs.maxHeight = solution.trajectory.getMaxHeight();
                outputs.trajectoryPoints = solution.trajectory.trajectory.size();
            }
        }
        outputs.calculationTimeMs = calculationTimeMs;
        return outputs;
    }
    
    /**
     * Convert CalibrationParameters to loggable data.
     */
    public static Calibration toCalibration(CalibrationParameters params, int shotCount, double hitRate) {
        Calibration data = new Calibration();
        data.dragCoefficient = params.dragCoefficient;
        data.magnusCoefficient = params.magnusCoefficient;
        data.speedEfficiency = params.speedEfficiency;
        data.spinEfficiency = params.spinEfficiency;
        data.restitutionCoefficient = params.restitutionCoefficient;
        data.frictionCoefficient = params.frictionCoefficient;
        data.shotCount = shotCount;
        data.hitRate = hitRate;
        return data;
    }
    
    /**
     * Create inputs from current state.
     */
    public static Inputs createInputs(double robotX, double robotY, double robotZ,
                                      double targetX, double targetY, double targetZ,
                                      double launchSpeed, double spinRate) {
        Inputs inputs = new Inputs();
        inputs.robotX = robotX;
        inputs.robotY = robotY;
        inputs.robotZ = robotZ;
        inputs.targetX = targetX;
        inputs.targetY = targetY;
        inputs.targetZ = targetZ;
        inputs.launchSpeed = launchSpeed;
        inputs.spinRate = spinRate;
        inputs.timestamp = System.currentTimeMillis();
        return inputs;
    }
}
