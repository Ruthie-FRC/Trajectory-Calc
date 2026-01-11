package frc.robot.trajectory.empirical;

import frc.robot.trajectory.Vector3D;
import java.io.*;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

/**
 * Records and analyzes real shot attempts to bridge ideal physics and real-world shooter behavior.
 * 
 * <p>This class maintains a database of shot attempts with their inputs (robot pose, turret/hood angles,
 * velocity, spin) and outcomes (made/missed, rim interactions, error measurements). Data can be used
 * to learn systematic biases and improve trajectory predictions.
 * 
 * <p>Thread-safe for concurrent recording from multiple sources.
 * 
 * <p>Example usage:
 * <pre>
 * ShotOutcomeModel model = new ShotOutcomeModel();
 * 
 * // Record a shot after it completes
 * model.recordShot(
 *     robotX, robotY, robotHeading,
 *     turretYaw, hoodPitch,
 *     shooterVelocity, spinRate, new Vector3D(0, 1, 0),
 *     distanceToTarget,
 *     ShotOutcome.RIM_IN,
 *     RimLocation.FRONT,
 *     0.05, -0.02  // lateral and vertical error in meters
 * );
 * 
 * // Export for offline analysis
 * model.exportToCSV("shot_data.csv");
 * 
 * // Analyze performance
 * double successRate = model.getSuccessRate();
 * List&lt;ShotRecord&gt; longShots = model.getShotsInRange(5.0, 8.0);
 * </pre>
 */
public class ShotOutcomeModel {
    
    private final List<ShotRecord> shots;
    private final Map<String, Object> metadata;
    
    /**
     * Creates a new empty shot outcome model.
     */
    public ShotOutcomeModel() {
        this.shots = Collections.synchronizedList(new ArrayList<>());
        this.metadata = new ConcurrentHashMap<>();
        this.metadata.put("created", System.currentTimeMillis());
        this.metadata.put("version", "1.0");
    }
    
    /**
     * Records a shot attempt with full input and outcome data.
     * 
     * @param robotX Robot X position in field coordinates (meters)
     * @param robotY Robot Y position in field coordinates (meters)
     * @param robotHeading Robot heading in field coordinates (radians)
     * @param turretYaw Turret yaw angle relative to robot (radians)
     * @param hoodPitch Hood pitch angle from horizontal (radians)
     * @param shooterVelocity Exit velocity from shooter wheels (m/s)
     * @param spinRate Ball spin rate (rad/s)
     * @param spinAxis Ball spin axis (unit vector)
     * @param distanceToTarget Distance from robot to target (meters)
     * @param outcome Shot outcome classification
     * @param rimLocation Rim contact location if applicable
     * @param lateralError Lateral error at target (meters, positive = right)
     * @param verticalError Vertical error at target (meters, positive = high)
     */
    public synchronized void recordShot(
            double robotX, double robotY, double robotHeading,
            double turretYaw, double hoodPitch,
            double shooterVelocity, double spinRate, Vector3D spinAxis,
            double distanceToTarget,
            ShotOutcome outcome, RimLocation rimLocation,
            double lateralError, double verticalError) {
        
        ShotRecord record = new ShotRecord(
            robotX, robotY, robotHeading,
            turretYaw, hoodPitch,
            shooterVelocity, spinRate, spinAxis,
            distanceToTarget,
            System.currentTimeMillis(),
            outcome, rimLocation,
            lateralError, verticalError
        );
        
        shots.add(record);
    }
    
    /**
     * Gets all recorded shots.
     * 
     * @return Unmodifiable list of shot records
     */
    public List<ShotRecord> getAllShots() {
        synchronized (shots) {
            return Collections.unmodifiableList(new ArrayList<>(shots));
        }
    }
    
    /**
     * Gets shots within a distance range.
     * 
     * @param minDistance Minimum distance (meters, inclusive)
     * @param maxDistance Maximum distance (meters, inclusive)
     * @return List of shots in range
     */
    public List<ShotRecord> getShotsInRange(double minDistance, double maxDistance) {
        synchronized (shots) {
            return shots.stream()
                    .filter(s -> s.distanceToTarget >= minDistance && s.distanceToTarget <= maxDistance)
                    .collect(Collectors.toList());
        }
    }
    
    /**
     * Calculates overall success rate (made shots + rim-ins).
     * 
     * @return Success rate from 0.0 to 1.0, or 0.0 if no shots recorded
     */
    public double getSuccessRate() {
        synchronized (shots) {
            if (shots.isEmpty()) return 0.0;
            
            long successes = shots.stream()
                    .filter(s -> s.outcome == ShotOutcome.MADE_SHOT || s.outcome == ShotOutcome.RIM_IN)
                    .count();
            
            return (double) successes / shots.size();
        }
    }
    
    /**
     * Calculates success rate within a distance range.
     * 
     * @param minDistance Minimum distance (meters)
     * @param maxDistance Maximum distance (meters)
     * @return Success rate for shots in range, or 0.0 if none
     */
    public double getSuccessRateInRange(double minDistance, double maxDistance) {
        List<ShotRecord> inRange = getShotsInRange(minDistance, maxDistance);
        if (inRange.isEmpty()) return 0.0;
        
        long successes = inRange.stream()
                .filter(s -> s.outcome == ShotOutcome.MADE_SHOT || s.outcome == ShotOutcome.RIM_IN)
                .count();
        
        return (double) successes / inRange.size();
    }
    
    /**
     * Gets the rim-out rate for a specific rim location.
     * 
     * @param location Rim location to analyze
     * @return Proportion of rim-outs at this location among all rim-outs
     */
    public double getRimOutRate(RimLocation location) {
        synchronized (shots) {
            long totalRimOuts = shots.stream()
                    .filter(s -> s.outcome == ShotOutcome.RIM_OUT)
                    .count();
            
            if (totalRimOuts == 0) return 0.0;
            
            long locationRimOuts = shots.stream()
                    .filter(s -> s.outcome == ShotOutcome.RIM_OUT && s.rimLocation == location)
                    .count();
            
            return (double) locationRimOuts / totalRimOuts;
        }
    }
    
    /**
     * Gets the number of recorded shots.
     * 
     * @return Total shot count
     */
    public int getShotCount() {
        return shots.size();
    }
    
    /**
     * Clears all recorded shots.
     */
    public synchronized void clear() {
        shots.clear();
    }
    
    /**
     * Exports shot data to CSV format.
     * 
     * @param filename Output file path
     * @throws IOException If file cannot be written
     */
    public void exportToCSV(String filename) throws IOException {
        synchronized (shots) {
            try (PrintWriter writer = new PrintWriter(new FileWriter(filename))) {
                // Header
                writer.println("timestamp,robotX,robotY,robotHeading,turretYaw,hoodPitch," +
                        "shooterVelocity,spinRate,spinAxisX,spinAxisY,spinAxisZ," +
                        "distanceToTarget,outcome,rimLocation,lateralError,verticalError");
                
                // Data rows
                for (ShotRecord shot : shots) {
                    writer.printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%s,%s,%.3f,%.3f%n",
                            shot.timestamp,
                            shot.robotX, shot.robotY, shot.robotHeading,
                            shot.turretYaw, shot.hoodPitch,
                            shot.shooterVelocity, shot.spinRate,
                            shot.spinAxis.x, shot.spinAxis.y, shot.spinAxis.z,
                            shot.distanceToTarget,
                            shot.outcome, shot.rimLocation,
                            shot.lateralError, shot.verticalError);
                }
            }
        }
    }
    
    /**
     * Imports shot data from CSV format.
     * 
     * @param filename Input file path
     * @throws IOException If file cannot be read
     */
    public void importFromCSV(String filename) throws IOException {
        synchronized (shots) {
            try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
                String line = reader.readLine(); // Skip header
                
                while ((line = reader.readLine()) != null) {
                    String[] parts = line.split(",");
                    if (parts.length < 16) continue;
                    
                    try {
                        ShotRecord record = new ShotRecord(
                                Double.parseDouble(parts[1]),  // robotX
                                Double.parseDouble(parts[2]),  // robotY
                                Double.parseDouble(parts[3]),  // robotHeading
                                Double.parseDouble(parts[4]),  // turretYaw
                                Double.parseDouble(parts[5]),  // hoodPitch
                                Double.parseDouble(parts[6]),  // shooterVelocity
                                Double.parseDouble(parts[7]),  // spinRate
                                new Vector3D(
                                        Double.parseDouble(parts[8]),
                                        Double.parseDouble(parts[9]),
                                        Double.parseDouble(parts[10])
                                ),
                                Double.parseDouble(parts[11]), // distanceToTarget
                                Long.parseLong(parts[0]),       // timestamp
                                ShotOutcome.valueOf(parts[12]), // outcome
                                RimLocation.valueOf(parts[13]), // rimLocation
                                Double.parseDouble(parts[14]),  // lateralError
                                Double.parseDouble(parts[15])   // verticalError
                        );
                        shots.add(record);
                    } catch (Exception e) {
                        // Skip malformed lines
                        System.err.println("Warning: Skipping malformed CSV line: " + line);
                    }
                }
            }
        }
    }
    
    /**
     * Exports shot data to JSON format.
     * 
     * @param filename Output file path
     * @throws IOException If file cannot be written
     */
    public void exportToJSON(String filename) throws IOException {
        synchronized (shots) {
            try (PrintWriter writer = new PrintWriter(new FileWriter(filename))) {
                writer.println("{");
                writer.println("  \"metadata\": {");
                writer.println("    \"version\": \"1.0\",");
                writer.println("    \"shotCount\": " + shots.size() + ",");
                writer.println("    \"exportTime\": " + System.currentTimeMillis());
                writer.println("  },");
                writer.println("  \"shots\": [");
                
                for (int i = 0; i < shots.size(); i++) {
                    ShotRecord shot = shots.get(i);
                    writer.println("    {");
                    writer.println("      \"timestamp\": " + shot.timestamp + ",");
                    writer.println("      \"robot\": {");
                    writer.println("        \"x\": " + shot.robotX + ",");
                    writer.println("        \"y\": " + shot.robotY + ",");
                    writer.println("        \"heading\": " + shot.robotHeading);
                    writer.println("      },");
                    writer.println("      \"turretYaw\": " + shot.turretYaw + ",");
                    writer.println("      \"hoodPitch\": " + shot.hoodPitch + ",");
                    writer.println("      \"shooterVelocity\": " + shot.shooterVelocity + ",");
                    writer.println("      \"spin\": {");
                    writer.println("        \"rate\": " + shot.spinRate + ",");
                    writer.println("        \"axis\": {");
                    writer.println("          \"x\": " + shot.spinAxis.x + ",");
                    writer.println("          \"y\": " + shot.spinAxis.y + ",");
                    writer.println("          \"z\": " + shot.spinAxis.z);
                    writer.println("        }");
                    writer.println("      },");
                    writer.println("      \"distanceToTarget\": " + shot.distanceToTarget + ",");
                    writer.println("      \"outcome\": \"" + shot.outcome + "\",");
                    writer.println("      \"rimLocation\": \"" + shot.rimLocation + "\",");
                    writer.println("      \"error\": {");
                    writer.println("        \"lateral\": " + shot.lateralError + ",");
                    writer.println("        \"vertical\": " + shot.verticalError);
                    writer.println("      }");
                    writer.print("    }");
                    if (i < shots.size() - 1) writer.println(",");
                    else writer.println();
                }
                
                writer.println("  ]");
                writer.println("}");
            }
        }
    }
    
    /**
     * Imports shot data from JSON format.
     * 
     * @param filename Input file path
     * @throws IOException If file cannot be read
     */
    public void importFromJSON(String filename) throws IOException {
        // Simple JSON parser for our specific format
        synchronized (shots) {
            try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
                String line;
                ShotRecord.Builder builder = null;
                
                while ((line = reader.readLine()) != null) {
                    line = line.trim();
                    
                    if (line.contains("\"timestamp\":")) {
                        builder = new ShotRecord.Builder();
                        builder.timestamp(parseLong(line));
                    } else if (builder != null) {
                        if (line.contains("\"x\":") && line.contains("robot")) {
                            builder.robotX(parseDouble(line));
                        } else if (line.contains("\"y\":") && !line.contains("axis")) {
                            builder.robotY(parseDouble(line));
                        } else if (line.contains("\"heading\":")) {
                            builder.robotHeading(parseDouble(line));
                        } else if (line.contains("\"turretYaw\":")) {
                            builder.turretYaw(parseDouble(line));
                        } else if (line.contains("\"hoodPitch\":")) {
                            builder.hoodPitch(parseDouble(line));
                        } else if (line.contains("\"shooterVelocity\":")) {
                            builder.shooterVelocity(parseDouble(line));
                        } else if (line.contains("\"rate\":")) {
                            builder.spinRate(parseDouble(line));
                        } else if (line.contains("\"x\":") && line.contains("axis")) {
                            // In spin axis
                            builder.spinAxisX(parseDouble(line));
                        } else if (line.contains("\"y\":") && line.contains("axis")) {
                            builder.spinAxisY(parseDouble(line));
                        } else if (line.contains("\"z\":")) {
                            builder.spinAxisZ(parseDouble(line));
                        } else if (line.contains("\"distanceToTarget\":")) {
                            builder.distanceToTarget(parseDouble(line));
                        } else if (line.contains("\"outcome\":")) {
                            builder.outcome(ShotOutcome.valueOf(parseString(line)));
                        } else if (line.contains("\"rimLocation\":")) {
                            builder.rimLocation(RimLocation.valueOf(parseString(line)));
                        } else if (line.contains("\"lateral\":")) {
                            builder.lateralError(parseDouble(line));
                        } else if (line.contains("\"vertical\":")) {
                            builder.verticalError(parseDouble(line));
                            // End of shot record
                            try {
                                shots.add(builder.build());
                            } catch (Exception e) {
                                System.err.println("Warning: Skipping incomplete JSON shot record");
                            }
                            builder = null;
                        }
                    }
                }
            }
        }
    }
    
    private double parseDouble(String line) {
        int colonIdx = line.indexOf(":");
        if (colonIdx < 0) return 0.0;
        String value = line.substring(colonIdx + 1).trim();
        value = value.replaceAll("[,}]", "");
        return Double.parseDouble(value);
    }
    
    private long parseLong(String line) {
        int colonIdx = line.indexOf(":");
        if (colonIdx < 0) return 0;
        String value = line.substring(colonIdx + 1).trim();
        value = value.replaceAll("[,}]", "");
        return Long.parseLong(value);
    }
    
    private String parseString(String line) {
        int colonIdx = line.indexOf(":");
        if (colonIdx < 0) return "";
        String value = line.substring(colonIdx + 1).trim();
        value = value.replaceAll("[\",}]", "").trim();
        return value;
    }
}
