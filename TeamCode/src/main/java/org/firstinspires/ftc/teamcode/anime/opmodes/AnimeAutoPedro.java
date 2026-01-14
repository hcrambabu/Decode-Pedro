package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "Anime: Pedro Auto Test", group = "Anime")
public class AnimeAutoPedro extends LinearOpMode {

    private Follower follower;
    private PathChain testPath;

    // #region agent log
    private void log(String location, String message, Object data, String hypothesisId) {
        telemetry.addData("DEBUG [" + hypothesisId + "]", location + ": " + message + (data != null ? " | " + data : ""));
        telemetry.update();
    }
    // #endregion

    @Override
    public void runOpMode() {
        // #region agent log
        log("AnimeAutoBlue.java:18", "runOpMode called", null, "A");
        // #endregion

        // --------------------
        // INIT
        // --------------------
        telemetry.addLine("Initializing...");
        telemetry.update();
        
        // #region agent log
        log("AnimeAutoPedro.java:37", "Before Follower initialization", "Constants.createFollower", "B");
        // #endregion
        follower = Constants.createFollower(hardwareMap);
        // #region agent log
        log("AnimeAutoPedro.java:40", "After Follower initialization", follower != null ? "success" : "null", "B");
        // #endregion

        // Starting pose (x, y, heading in radians)
        Pose startPose = new Pose(56.00, 8.00, Math.PI/2);
        // #region agent log
        log("AnimeAutoPedro.java:45", "Before setStartingPose", "Pose(56.00, 8.00, PI/2)", "E");
        // #endregion
        follower.setStartingPose(startPose);
        follower.update();
        // #region agent log
        log("AnimeAutoPedro.java:49", "After setStartingPose", "success", "E");
        // #endregion

        // Build a simple path using BezierLine
        Pose endPose = new Pose(92.60806916426513, 83.11815561959654, Math.PI/2);
        // #region agent log
        log("AnimeAutoPedro.java:54", "Before pathBuilder", null, "C");
        // #endregion
        testPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
        // #region agent log
        log("AnimeAutoPedro.java:59", "After pathBuilder.build()", testPath != null ? "success" : "null", "C");
        // #endregion

        telemetry.addLine("Pedro Auto Initialized");
        telemetry.addData("Start Pose", String.format("(%.2f, %.2f, %.2f°)", 
            startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
        telemetry.addData("End Pose", String.format("(%.2f, %.2f, %.2f°)", 
            endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading())));
        telemetry.update();

        // --------------------
        // WAIT FOR START
        // --------------------
        waitForStart();
        if (isStopRequested()) return;

        // --------------------
        // RUN PATH
        // --------------------
        if (testPath == null) {
            // #region agent log
            log("AnimeAutoPedro.java:78", "ERROR: testPath is null", null, "D");
            // #endregion
            telemetry.addLine("ERROR: Path is null!");
            telemetry.update();
            while (opModeIsActive()) {
                idle();
            }
            return;
        }
        
        // #region agent log
        log("AnimeAutoPedro.java:87", "Before followPath", "path exists", "D");
        // #endregion
        follower.followPath(testPath);
        // #region agent log
        log("AnimeAutoPedro.java:90", "After followPath", "isBusy=" + follower.isBusy(), "D");
        // #endregion

        int loopCount = 0;
        // #region agent log
        log("AnimeAutoPedro.java:94", "Entering path following loop", "isBusy=" + follower.isBusy(), "D");
        // #endregion
        
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            telemetry.addData("Pose X", String.format("%.2f", follower.getPose().getX()));
            telemetry.addData("Pose Y", String.format("%.2f", follower.getPose().getY()));
            telemetry.addData("Heading", String.format("%.2f", Math.toDegrees(follower.getPose().getHeading())));
            telemetry.addData("Loop Count", loopCount);
            telemetry.addData("Is Busy", follower.isBusy());
            
            // Log every 50 iterations to avoid spam
            if (loopCount % 50 == 0) {
                // #region agent log
                log("AnimeAutoPedro.java:108", "Loop iteration", "count=" + loopCount + ", isBusy=" + follower.isBusy(), "D");
                // #endregion
            }
            
            telemetry.update();
            loopCount++;
        }
        
        // #region agent log
        log("AnimeAutoPedro.java:118", "Loop exited", "final count=" + loopCount + ", isBusy=" + follower.isBusy(), "D");
        // #endregion
        
        telemetry.addLine("Path following complete");
        telemetry.addData("Final Loop Count", loopCount);
        telemetry.addData("Final Is Busy", follower.isBusy());
        telemetry.addData("Final Pose X", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("Final Pose Y", String.format("%.2f", follower.getPose().getY()));
        telemetry.update();

        // --------------------
        // END
        // --------------------
        while (opModeIsActive()) {
            idle();
        }
    }
}
