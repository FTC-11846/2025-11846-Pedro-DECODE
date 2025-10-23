package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.MainCharacter;

/**
 * RobotState - Static state persistence for Auto → TeleOp transition
 * 
 * FTC allows Auto OpModes to preselect TeleOp OpModes for the driver period.
 * This class stores robot state that needs to transfer between OpModes:
 * - Which robot is active (so driver doesn't need to reselect)
 * - Last known pose (for seamless odometry continuation)
 * - Alliance selection (for field-aware logic)
 * 
 * Usage:
 * - Auto: Sets values at end of autonomous
 * - TeleOp: Reads values during init to skip selection if available
 * 
 * Note: Static variables persist for the lifetime of the app process,
 * surviving OpMode switches. They reset when app restarts.
 */
public class RobotState {
    
    // Robot configuration
    public static MainCharacter activeRobot = null;
    
    // Alliance selection
    public static Alliance activeAlliance = null;
    
    // Last known pose from autonomous
    public static Pose lastKnownPose = null;
    
    // Flag indicating if state is valid (set by Auto, cleared by TeleOp after use)
    public static boolean hasValidState = false;
    
    /**
     * Store state at end of autonomous
     * Call this from Auto's stop() or final state
     */
    public static void saveState(MainCharacter robot, Alliance alliance, Pose pose) {
        activeRobot = robot;
        activeAlliance = alliance;
        lastKnownPose = pose;
        hasValidState = true;
    }
    
    /**
     * Restore state in TeleOp init
     * Returns true if valid state was available
     */
    public static boolean hasState() {
        return hasValidState && activeRobot != null;
    }
    
    /**
     * Clear state after TeleOp has consumed it
     * Prevents stale state on next run
     */
    public static void clearState() {
        activeRobot = null;
        activeAlliance = null;
        lastKnownPose = null;
        hasValidState = false;
    }
    
    /**
     * Get a debug string of current state
     */
    public static String getStateString() {
        if (!hasValidState) {
            return "No valid state";
        }
        return String.format("Robot: %s, Alliance: %s, Pose: X=%.1f Y=%.1f H=%.1f°",
            activeRobot != null ? activeRobot.toString() : "null",
            activeAlliance != null ? activeAlliance.toString() : "null",
            lastKnownPose != null ? lastKnownPose.getX() : 0,
            lastKnownPose != null ? lastKnownPose.getY() : 0,
            lastKnownPose != null ? Math.toDegrees(lastKnownPose.getHeading()) : 0);
    }
}
