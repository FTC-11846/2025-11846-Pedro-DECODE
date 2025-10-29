package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * StartingPosition - NEAR or FAR starting positions
 * 
 * IMPORTANT: All positions use PEDRO COORDINATES!
 * - Origin: Center of field (72, 72)
 * - X-axis: Increases toward red wall
 * - Y-axis: Increases toward far side  
 * - Heading: 0° = +X, 90° = +Y, 180° = -X, 270° = -Y
 * 
 * Reference: https://pedropathing.com/docs/pathing/reference/coordinates
 * 
 * Positions are defined in BLUE alliance coordinates.
 * Use FieldMirror.mirrorPose() to get RED alliance positions.
 */
public enum StartingPosition {
    NEAR("Near"),
    FAR("Far");
    
    private final String displayName;
    
    StartingPosition(String displayName) {
        this.displayName = displayName;
    }
    
    /**
     * Get the BLUE alliance pose for this position (PEDRO COORDINATES)
     * RED poses are calculated via FieldMirror.mirrorPose()
     * 
     * Pedro Coordinate System:
     * - Field dimensions: 144" x 144"
     * - Center: (72, 72)
     * - Blue wall: X ≈ 86 (start line)
     * - Red wall: X ≈ 58 (mirrored)
     * - Audience side: Y ≈ 8
     * - Far side: Y ≈ 136
     * 
     * Headings (Pedro standard):
     * - 0° = Facing +X (toward red wall)
     * - 90° = Facing +Y (toward far side) 
     * - 180° = Facing -X (toward blue wall)
     * - 270° = Facing -Y (toward audience)
     */
    public Pose getBluePose() {
        switch (this) {
            case NEAR:
                // Blue near position: audience side, facing toward far side
                return new Pose(86, 8, Math.toRadians(90));
            case FAR:
                // Blue far position: far side, facing toward audience
                return new Pose(86, 136, Math.toRadians(270));
            default:
                throw new IllegalStateException("Unknown starting position: " + this);
        }
    }
    
    @Override
    public String toString() {
        return displayName;
    }
}

/*
 * TO ADD MORE STARTING POSITIONS:
 * 
 * 1. Add new enum value:
 *    MIDDLE("Middle"),
 * 
 * 2. Add case in getBluePose() with PEDRO COORDINATES:
 *    case MIDDLE:
 *        return new Pose(86, 72, Math.toRadians(0));  // Center, facing red
 * 
 * 3. Done! Alliance mirroring and OpMode selection handle the rest automatically.
 * 
 * Example for 4 positions (NEAR, MIDDLE, FAR, CORNER):
 * 
 * public enum StartingPosition {
 *     NEAR("Near"),
 *     MIDDLE("Middle"),
 *     FAR("Far"),
 *     CORNER("Corner");
 *     
 *     public Pose getBluePose() {
 *         switch (this) {
 *             case NEAR: return new Pose(86, 8, Math.toRadians(90));
 *             case MIDDLE: return new Pose(86, 72, Math.toRadians(0));
 *             case FAR: return new Pose(86, 136, Math.toRadians(270));
 *             case CORNER: return new Pose(86, 136, Math.toRadians(315));
 *             default: throw new IllegalStateException("Unknown position");
 *         }
 *     }
 * }
 * 
 * REMEMBER: Always use Pedro coordinates! Test with Pedro Visualizer:
 * https://visualizer.pedropathing.com/
 */
