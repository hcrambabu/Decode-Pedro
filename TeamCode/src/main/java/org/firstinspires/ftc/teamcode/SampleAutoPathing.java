package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
@TeleOp
public class SampleAutoPathing extends OpMode{
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
//    private Follower follower;
//    private Timer pathTimer, opModeTimer;
//
//    public enum PathState{
//        // START POSITION_END POSITION
//        // DRIVE > MOVEMENT STATE
//        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
//        DRIVE_STARTPOS_SHOOT_POS,
//        SHOOT_PRELOAD
//
//    }
//
//    PathState pathState;
//
//    private final Pose startpose = new Pose(21.08676951993108, 123.31109405591667, Math.toRadians(145));
//    private final Pose shootPose = new Pose(56.496627770381394, 86.30978150207535, Math.toRadians(145));
//
//    private final Pose endPose = new Pose(61.27099068055448, 109.3858689012452, Math.toRadians(180));
//    private PathChain driveStartPosShootPos;
//
//    public void buildPaths(){
//        //put in coordinates for starting pose > ending pose
//        driveStartPosShootPos = follower.pathBuilder()
//                .addPath(new BezierLine(startpose, shootPose))
//                .setLinearHeadingInterpolation(startpose.getHeading(), shootPose.getHeading())
//                .build();
//    }
//
//    public void statePathUpdate(){
//        switch(pathState){
//            case DRIVE_STARTPOS_SHOOT_POS:
//                follower.followPath(driveStartPosShootPos, true);
//                setPathState(PathState.SHOOT_PRELOAD);
//                break;
//
//            case SHOOT_PRELOAD:
//                //check is follower done it's path
//                if(!follower.isBusy()){
//                    //TODO add logic to flywheel shooter
//                    telemetry.addLine("Done Path 1");
//                }
//                break;
//            default:
//                telemetry.addLine("No State Commanded");
//                break;
//        }
//    }
//
//    public void setPathState(PathState newState){
//        pathState = newState;
//        pathTimer.resetTimer();
//    }
//    @Override
//    public void init() {
//        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
//        pathTimer = new Timer();
//        opModeTimer = new Timer();
//        opModeTimer.resetTimer();
//        // TODO add in any other init mechanisms
//
//        buildPaths();
//        follower.setPose(startpose);
//    }
//    public void start(){
//        opModeTimer.resetTimer();
//        setPathState(pathState);
//    }
//    @Override
//    public void loop() {
//        follower.update();
//        statePathUpdate();
//        telemetry.addData("path state", pathState.toString());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
//    }
}
