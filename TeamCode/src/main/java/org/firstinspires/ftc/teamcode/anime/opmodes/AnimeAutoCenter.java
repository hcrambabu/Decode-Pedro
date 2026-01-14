package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.path.Path;
import com.pedropathing.path.PathBuilder;
import com.pedropathing.path.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Anime: Pedro Auto Test", group = "Anime")
public class AnimeAutoPedro extends LinearOpMode {

    private Follower follower;
    private Path testPath;

    @Override
    public void runOpMode() {

        follower = new Follower(hardwareMap);

        follower.setStartingPose(new Pose(56, 8, Math.PI / 2));

        testPath = new PathBuilder()
                .addPoint(new Point(56, 8))
                .addPoint(new Point(92.6, 83.1))
                .build();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        follower.followPath(testPath);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }

        while (opModeIsActive()) idle();
    }
}
