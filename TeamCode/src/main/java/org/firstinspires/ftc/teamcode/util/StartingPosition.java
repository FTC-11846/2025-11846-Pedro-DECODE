package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * StartingPosition - NEAR or FAR starting positions
 * Positions are defined in BLUE alliance coordinates
 * Use FieldMirror.mirrorPose() to get RED alliance positions
 */
public enum StartingPosition {
    NEAR("Near"),
    FAR("Far");
    
    private final String displayName;
    
    StartingPosition(String displayName) {
        this.displayName = displayName;
    }
    
    /**
     * Get the BLUE alliance pose for this position
     * RED poses are calculated via FieldMirror.mirrorPose()
     * 
     * Field coordinate system (BLUE perspective):
     * - X: 0 = blue wall, 144 = red wall
     * - Y: 0 = audience side, 144 = far side
     * - Heading: 180° = facing red wall (toward opponent)
     */
    public Pose getBluePose() {
        switch (this) {
            case NEAR:
                // Blue near position: audience side
                return new Pose(86, 8, Math.toRadians(180));
            case FAR:
                // Blue far position: far side
                return new Pose(86, 136, Math.toRadians(180));
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
 * 2. Add case in getBluePose():
 *    case MIDDLE:
 *        return new Pose(86, 72, Math.toRadians(180));
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
 *             case NEAR: return new Pose(86, 8, Math.toRadians(180));
 *             case MIDDLE: return new Pose(86, 72, Math.toRadians(180));
 *             case FAR: return new Pose(86, 136, Math.toRadians(180));
 *             case CORNER: return new Pose(86, 136, Math.toRadians(225));
 *             default: throw new IllegalStateException("Unknown position");
 *         }
 *     }
 * }
 */
