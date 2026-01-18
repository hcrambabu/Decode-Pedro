package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.anime.robot.AutoBase;

@Configurable
@Autonomous(name="Anime: Auto Red Bottom", group="Anime")
public class AnimeAutoRedBottom extends AutoBase {

    @Override
    public void assignPosesToVariables() {
        startPose = new Pose(87, 9, Math.toRadians(90));
        endPose = new Pose(39, 33, Math.toRadians(90));
        shootPose = new Pose(87, 15, Math.toRadians(60));

        pickupOrderPoses = new Pose[]{
                new Pose(100, 35.5, Math.toRadians(0)),
                new Pose(100, 59.5, Math.toRadians(0)),
                new Pose(100, 83.5, Math.toRadians(0))
        };
        isRed = true;
        shootingVelocity = 2000;
    }
}

