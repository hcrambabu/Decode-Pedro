package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.anime.robot.AutoBase;

@Configurable
@Autonomous(name="Anime: Auto Red Top", group="Anime")
public class AnimeAutoRedTop extends AutoBase {

    @Override
    public void assignPosesToVariables() {
        startPose = new Pose(87, 135, Math.toRadians(270));
        endPose = new Pose(39, 33, Math.toRadians(90));
        shootPose = new Pose(81, 57, Math.toRadians(45));

        pickupOrderPoses = new Pose[]{
                new Pose(100, 83.5, Math.toRadians(0)),
                new Pose(100, 59.5, Math.toRadians(0)),
                new Pose(100, 35.5, Math.toRadians(0))
        };
        isRed = true;
        shootingVelocity = 2000;
    }
}

