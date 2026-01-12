package frc.robot.trajectory.physics;

import frc.robot.trajectory.util.Vector3D;

/**
 * HUB target geometry and collision detection.
 * Models the hexagonal opening and rectangular structure.
 */
public class HubGeometry {
    private final Vector3D center;
    private final double openingFlatToFlat;
    private final double openingHeight;
    private final double footprintSize;
    
    public HubGeometry() {
        this(new Vector3D(PhysicsConstants.DEFAULT_HUB_CENTER_X,
                         PhysicsConstants.DEFAULT_HUB_CENTER_Y,
                         PhysicsConstants.DEFAULT_HUB_OPENING_HEIGHT),
             PhysicsConstants.DEFAULT_HUB_OPENING_FLAT_TO_FLAT,
             PhysicsConstants.DEFAULT_HUB_OPENING_HEIGHT,
             PhysicsConstants.DEFAULT_HUB_FOOTPRINT_SIZE);
    }
    
    public HubGeometry(Vector3D center, double openingFlatToFlat, 
                       double openingHeight, double footprintSize) {
        this.center = center;
        this.openingFlatToFlat = openingFlatToFlat;
        this.openingHeight = openingHeight;
        this.footprintSize = footprintSize;
    }
    
    /**
     * Check if a point is inside the hexagonal opening (2D check in XY plane).
     * Uses the flat-to-flat distance to determine hexagon bounds.
     */
    public boolean isInsideHexagon(double x, double y) {
        // Translate to hub-centered coordinates
        double dx = x - center.x;
        double dy = y - center.y;
        
        // For a regular hexagon with flat-to-flat distance d,
        // the hexagon can be approximated by checking distance from center
        // and checking against 6 half-planes
        double radius = openingFlatToFlat / 2.0;
        
        // Simple approximation: use inscribed circle
        // A more accurate check would test against all 6 edges
        double distFromCenter = Math.sqrt(dx * dx + dy * dy);
        
        // For hexagon, we need to check against the flat edges
        // Using a conservative circular approximation for now
        return distFromCenter <= radius * 0.9; // Conservative estimate
    }
    
    /**
     * Check if a ball at given position would clear the opening with margin.
     * Uses default ball radius.
     */
    public boolean hasEnoughClearance(Vector3D position, double marginMultiplier) {
        double requiredMargin = PhysicsConstants.DEFAULT_BALL_RADIUS * marginMultiplier;
        double effectiveRadius = (openingFlatToFlat / 2.0) - requiredMargin;
        
        double dx = position.x - center.x;
        double dy = position.y - center.y;
        double distFromCenter = Math.sqrt(dx * dx + dy * dy);
        
        return distFromCenter <= effectiveRadius;
    }
    
    /**
     * Check if a ball at given position would clear the opening with margin.
     * Uses specified ball radius.
     */
    public boolean hasEnoughClearance(Vector3D position, double marginMultiplier, double ballRadius) {
        double requiredMargin = ballRadius * marginMultiplier;
        double effectiveRadius = (openingFlatToFlat / 2.0) - requiredMargin;
        
        double dx = position.x - center.x;
        double dy = position.y - center.y;
        double distFromCenter = Math.sqrt(dx * dx + dy * dy);
        
        return distFromCenter <= effectiveRadius;
    }
    
    /**
     * Check if the ball has crossed the opening plane (z = opening height).
     */
    public boolean hasCrossedOpeningPlane(ProjectileState state) {
        return state.position.z <= openingHeight && state.velocity.z < 0;
    }
    
    /**
     * Check if the ball position is at or near the opening plane.
     */
    public boolean isAtOpeningPlane(ProjectileState state, double tolerance) {
        return Math.abs(state.position.z - openingHeight) < tolerance;
    }
    
    /**
     * Evaluate entry quality - returns a score (higher is better).
     * Considers entry angle, vertical velocity, and position margin.
     * OPTIMIZED FOR CENTER SHOTS - heavily rewards aiming for center.
     */
    public double evaluateEntry(ProjectileState state) {
        // Distance from center (in XY plane)
        double dx = state.position.x - center.x;
        double dy = state.position.y - center.y;
        double distFromCenter = Math.sqrt(dx * dx + dy * dy);
        
        // Margin score (HEAVILY prefer center entries)
        // Use exponential falloff to strongly favor center shots
        double maxRadius = openingFlatToFlat / 2.0;
        double normalizedDist = distFromCenter / maxRadius;
        
        // Exponential scoring: center (0) gets 1.0, edge (1.0) gets 0.0
        // Using quadratic falloff for strong center preference
        double marginScore = Math.max(0, 1.0 - normalizedDist * normalizedDist);
        
        // Extra bonus for being in center 40% of opening
        double centerBonus = 0.0;
        if (distFromCenter < maxRadius * 0.4) {
            centerBonus = 0.2 * (1.0 - (distFromCenter / (maxRadius * 0.4)));
        }
        
        // Velocity score (prefer good downward velocity)
        double vzScore = 0.0;
        if (state.velocity.z < 0) {
            vzScore = Math.min(1.0, -state.velocity.z / PhysicsConstants.DEFAULT_MAX_VERTICAL_VELOCITY);
        }
        
        // Entry angle score (prefer near-vertical for accuracy)
        double speed = state.velocity.magnitude();
        double entryAngle = 0;
        if (speed > 0.1) {
            entryAngle = Math.abs(Math.toDegrees(Math.asin(-state.velocity.z / speed)));
        }
        double angleScore = Math.max(0, 1.0 - entryAngle / PhysicsConstants.DEFAULT_MAX_ENTRY_ANGLE_DEG);
        
        // Combined score - heavily weighted toward center position (60%)
        return marginScore * 0.50 + centerBonus + vzScore * 0.25 + angleScore * 0.25;
    }
    
    public Vector3D getCenter() {
        return center;
    }
    
    public double getOpeningHeight() {
        return openingHeight;
    }
    
    public double getOpeningRadius() {
        return openingFlatToFlat / 2.0;
    }
    
    public double getOpeningFlatToFlat() {
        return openingFlatToFlat;
    }
}
