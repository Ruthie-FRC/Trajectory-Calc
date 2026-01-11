package frc.robot.trajectory.util.empirical;


/**
 * Classification of shot outcomes.
 */
public enum ShotOutcome {
    /** Shot went cleanly through the target without touching the rim */
    MADE_SHOT,
    
    /** Shot touched the rim but went in */
    RIM_IN,
    
    /** Shot touched the rim and bounced out */
    RIM_OUT,
    
    /** Shot flew over the target */
    OVERSHOOT,
    
    /** Shot fell short of the target */
    UNDERSHOOT,
    
    /** Shot missed the target in an unclassified way */
    MISSED
}
