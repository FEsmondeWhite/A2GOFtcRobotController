package org.firstinspires.ftc.teamcode.gobot;

import com.pedropathing.geometry.Pose;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class PoseStorage {
    public static Pose currentPose; // This will be null until it is set during an OpMode (like during auto).
    public static double imuOffset;
}
