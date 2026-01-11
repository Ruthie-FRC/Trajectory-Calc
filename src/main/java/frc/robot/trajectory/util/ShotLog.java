package frc.robot.trajectory;

/**
 * Logged shot data for calibration and learning.
 */
public class ShotLog {
    public final Vector3D robotPose;
    public final double launchYawDeg;
    public final double launchPitchDeg;
    public final double launchSpeed;
    public final double spinRate;
    public final boolean hit;
    public final long timestamp;
    
    public ShotLog(Vector3D robotPose, double launchYawDeg, double launchPitchDeg,
                   double launchSpeed, double spinRate, boolean hit) {
        this.robotPose = robotPose;
        this.launchYawDeg = launchYawDeg;
        this.launchPitchDeg = launchPitchDeg;
        this.launchSpeed = launchSpeed;
        this.spinRate = spinRate;
        this.hit = hit;
        this.timestamp = System.currentTimeMillis();
    }
    
    @Override
    public String toString() {
        return String.format("ShotLog[pose=%s, yaw=%.2f°, pitch=%.2f°, speed=%.2f m/s, spin=%.1f rad/s, hit=%s]",
            robotPose, launchYawDeg, launchPitchDeg, launchSpeed, spinRate, hit);
    }
}
