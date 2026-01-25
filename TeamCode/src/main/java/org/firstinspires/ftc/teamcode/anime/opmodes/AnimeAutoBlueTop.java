package org.firstinspires.ftc.teamcode.anime.opmodes;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.anime.robot.AutoBase;
import org.firstinspires.ftc.teamcode.anime.robot.PoseStorage;


@Configurable
@Autonomous(name="Anime: Auto Blue Top", group="Anime")
public class AnimeAutoBlueTop extends AutoBase {


    @Override
    public void assignPosesToVariables() {
        startPose = new Pose(56, 136, Math.toRadians(270));
        endPose = new Pose(5, 136, Math.toRadians(270));
        shootPose = new Pose(59, 84, Math.toRadians(110));
        pickupOrderPoses = new Pose[]{
                new Pose(44, 85, Math.toRadians(180)),
                new Pose(44, 60, Math.toRadians(180)),
//                new Pose(44, 36, Math.toRadians(180))
        };
        isRed = false;
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
