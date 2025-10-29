package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;

/**
 * Alliance - Red or Blue team selection
 * Used for field mirroring and alliance-specific logic
 */
public enum Alliance {
    RED("Red", Color.RED),
    BLUE("Blue", Color.BLUE);
    
    private final String displayName;
    private final int color;
    
    Alliance(String displayName, int color) {
        this.displayName = displayName;
        this.color = color;
    }
    
    /**
     * Get display name for UI
     */
    @Override
    public String toString() {
        return displayName;
    }
    
    /**
     * Get Android color for this alliance
     */
    public int getColor() {
        return color;
    }
    
    /**
     * Check if this is red alliance
     */
    public boolean isRed() {
        return this == RED;
    }
    
    /**
     * Check if this is blue alliance
     */
    public boolean isBlue() {
        return this == BLUE;
    }
}
