package org.firstinspires.ftc.teamcode.anime.robot;

import static org.firstinspires.ftc.teamcode.anime.robot.AutoBase.PPG_TAG_ID;

import com.pedropathing.geometry.Pose;

import java.util.Map;

public class PoseStorage {
    // Start with a default pose (0,0,0) to prevent null pointer exceptions
    // if TeleOp is run without running Auto first.
    public static Pose currentPose = new Pose(0, 0, 0);

    public static int pattrenNumber = AutoBase.PPG_TAG_ID;
    public static Map<Integer, BallColor[]> patternMap = Map.of(
            AutoBase.PPG_TAG_ID, new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN},
            AutoBase.PGP_TAG_ID, new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE},
            AutoBase.GPP_TAG_ID, new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE}
    );
}
