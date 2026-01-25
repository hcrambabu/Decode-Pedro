package org.firstinspires.ftc.teamcode.anime.opmodes;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.anime.robot.AutoBase;
import org.firstinspires.ftc.teamcode.anime.robot.PoseStorage;


@Configurable
@Autonomous(name="Anime: Auto Red Top", group="Anime")
public class AnimeAutoRedTop extends AutoBase {


    @Override
    public void assignPosesToVariables() {
        startPose = new Pose(96, 136, Math.toRadians(270));
        endPose = new Pose(90, 136, Math.toRadians(90));
        shootPose = new Pose(84, 84, Math.toRadians(420));


        pickupOrderPoses = new Pose[]{
                new Pose(100, 84, Math.toRadians(0)),
                new Pose(100, 59.5, Math.toRadians(0)),
                new Pose(100, 35.5, Math.toRadians(0))
        };
        isRed = true;
        shootingVelocity = 1330;
    }

    @Override
    public void loop() {
        // Update robot follower and movement
        follower.update();
        autonomousPathUpdate();
        robot.update();

        // Store current robot pose
        PoseStorage.currentPose = follower.getPose();

        // Run intake during APPROACH and PICKUP
        if (actionState == ActionState.PICKUP || actionState == ActionState.GO_TO_PICKUP_POSE) {
            if (!robot.isIntaking()) {
                robot.startIntake();
            }
        } else {
            robot.stopIntake();
        }

        // Update telemetry
        updateTelemetry();
    }
}
