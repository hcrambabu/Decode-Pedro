package org.firstinspires.ftc.teamcode.anime.robot;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    // Start with a default pose (0,0,0) to prevent null pointer exceptions
    // if TeleOp is run without running Auto first.
    public static Pose currentPose = new Pose(0, 0, 0);
}
