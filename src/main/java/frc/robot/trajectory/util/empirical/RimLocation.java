package frc.robot.trajectory.util.empirical;


/**
 * Location of rim contact for rim-out shots.
 */
public enum RimLocation {
    /** Front edge of rim (closer to shooter) */
    FRONT,
    
    /** Back edge of rim (far side from shooter) */
    BACK,
    
    /** Side edge of rim */
    SIDE,
    
    /** Rim contact location could not be determined */
    UNKNOWN,
    
    /** No rim contact */
    NONE
}
