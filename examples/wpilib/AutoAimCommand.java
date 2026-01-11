package examples.wpilib;

import com.ruthiefrc.trajectory.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Example command for auto-aiming at the target.
 * 
 * This command continuously calculates and updates the shooter aim
 * based on the robot's current position.
 */
public class AutoAimCommand extends CommandBase {
    
    private final ExampleShooterSubsystem shooter;
    private final java.util.function.Supplier<Pose2d> poseSupplier;
    
    private InverseSolver.SolutionResult currentSolution;
    
    /**
     * Create auto-aim command.
     * 
     * @param shooter Shooter subsystem
     * @param poseSupplier Function that returns current robot pose (e.g., drivetrain::getPose)
     */
    public AutoAimCommand(ExampleShooterSubsystem shooter, 
                          java.util.function.Supplier<Pose2d> poseSupplier) {
        this.shooter = shooter;
        this.poseSupplier = poseSupplier;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        currentSolution = null;
    }
    
    @Override
    public void execute() {
        // Get current robot position
        Pose2d currentPose = poseSupplier.get();
        
        // Calculate shooting solution
        currentSolution = shooter.calculateShot(currentPose);
        
        // Apply to mechanisms
        shooter.aimAtTarget(currentSolution);
    }
    
    @Override
    public boolean isFinished() {
        // Command runs until interrupted
        // Could add logic to finish when aimed and ready
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Optional: Stop motors or reset positions
    }
    
    /**
     * Check if we have a valid shooting solution.
     */
    public boolean hasValidSolution() {
        return currentSolution != null && currentSolution.isHit();
    }
    
    /**
     * Get the current solution quality score.
     */
    public double getSolutionScore() {
        return currentSolution != null ? currentSolution.score : 0.0;
    }
}
