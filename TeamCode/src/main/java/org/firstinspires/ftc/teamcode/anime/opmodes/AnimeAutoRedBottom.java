package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.anime.robot.AutoBase;
import org.firstinspires.ftc.teamcode.anime.robot.PoseStorage;

@Configurable
@Autonomous(name="Anime: Auto Red Bottom", group="Anime")
public class AnimeAutoRedBottom extends AutoBase {

    @Override
    public void assignPosesToVariables() {
        // Starting pose of the robot (match visualizer start point)
        startPose = new Pose(96, 8, Math.toRadians(90));

        // Shoot pose (same spot every time)
        shootPose = new Pose(96, 14, Math.toRadians(60));

        // Optional end pose
        endPose = new Pose(96, 96, Math.toRadians(45));

        // Pickup poses in order (match visualizer paths)
        pickupOrderPoses = new Pose[]{
                new Pose(100, 35.5, Math.toRadians(0)),
                new Pose(100, 59.5, Math.toRadians(0)),
                new Pose(100, 83.5, Math.toRadians(0)),
        };

        isRed = true;               // Team color
        shootingVelocity = 1700;    // Flywheel velocity
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
