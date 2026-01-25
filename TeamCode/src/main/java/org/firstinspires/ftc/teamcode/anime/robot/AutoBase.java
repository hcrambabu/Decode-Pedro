package org.firstinspires.ftc.teamcode.anime.robot;

import static org.firstinspires.ftc.teamcode.anime.robot.PoseStorage.pattrenNumber;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public abstract class AutoBase extends OpMode {

    private static final double ACTION_TIMEOUT = 4.0;
    private static final double PICKUP_DISTANCE = 45;
    private static final double INTAKE_TIME = 3.0;
    protected enum ActionState {
        SHOOT,
        GO_TO_PICKUP_POSE,
        PICKUP,
        GO_TO_SHOOT_POSE,
        GO_TO_END,
    }


    //set April Tag values to specific patterns
    public static final int PPG_TAG_ID = 23;
    public static final int PGP_TAG_ID = 22;
    public static final int GPP_TAG_ID = 21;
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;

    // Initialize poses
    protected Pose startPose; // Start Pose of our robot.
    protected Pose endPose;
    protected Pose shootPose; // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    protected Pose[] pickupOrderPoses; // Order of poses to pick up balls.
    protected boolean isRed = true;

    protected double shootingVelocity;

    private Path shootPreload;
    private PathChain[] pickupOrderPathChains;
    private PathChain[] shootPathChains;
    private PathChain[] pickupEndPathChains;
    private PathChain endPathChain;

    protected Follower follower;
    protected Timer pathTimer, actionTimer, opmodeTimer;
    protected int pathState;
    protected ActionState actionState;
    protected Limelight limelight;
    protected GreenApple robot;

    public abstract void assignPosesToVariables();

    @Override
    public void init() {
        Log.i("AutoBase", "Autonomous initializing");
        assignPosesToVariables();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        this.robot = new GreenApple(hardwareMap, telemetry, 8, true);
        this.limelight = robot.getLimelight();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        robot.setFlyWheelShootingVelocity(shootingVelocity);
        robot.getIndexer().setAllBallsTrue();
        buildPaths();
    }

    @Override
    public void start() {
        Log.i("AutoBase", "Autonomous started");
        opmodeTimer.resetTimer();
        findPattern();
        limelight.changePipeline(9); // Switch to driver cam after finding pattern
        setPathState(0);
        setActionState(ActionState.SHOOT);
        robot.startShooting();
    }

//    @Override
//    public void loop() {
//        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
//        autonomousPathUpdate();
//        robot.update();
//        PoseStorage.currentPose = follower.getPose();
//
//        // Feedback to Driver Hub for debugging
//        updateTelemetry();
//    }


    public final void idle() {
        Thread.yield();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void updateTelemetry() {
        telemetry.addData("April Tag Pattern", pattrenNumber);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Action State", actionState.name());
        telemetry.addData("Action Time", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Robot Pose", follower.getPose().toString());
        telemetry.addData("Follower Busy?", follower.isBusy());
        telemetry.update();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setActionState(ActionState state) {
        actionState = state;
        actionTimer.resetTimer();
    }

    public void findPattern() {
        for(int i = 0; i < 5; i++) {
            boolean foundTag = false;
            List<Integer> aprilTagIds = limelight.getAllAprilTagId();
            for (int aprilTagId : aprilTagIds) {
                telemetry.addData("Detected AprilTag ID", aprilTagId);
                if (aprilTagId >= GPP_TAG_ID && aprilTagId <= PPG_TAG_ID) {
                    pattrenNumber = aprilTagId;
                    foundTag = true;
                    break;
                }
            }
            if (foundTag) {
                break;
            }
            idle();
        }
        telemetry.addData("Detected AprilTag ID: ", pattrenNumber);
    }

    public void buildPaths() {
        shootPreload = new Path(new BezierLine(startPose, shootPose));
        shootPreload.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        pickupOrderPathChains = new PathChain[pickupOrderPoses.length];
        pickupEndPathChains = new PathChain[pickupOrderPoses.length];
        shootPathChains = new PathChain[pickupOrderPoses.length];
        for (int i = 0; i < pickupOrderPoses.length; i++) {
            Pose prevPose = i == 0 ? shootPose : pickupOrderPoses[i - 1];
            pickupOrderPathChains[i] = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, pickupOrderPoses[i]))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), pickupOrderPoses[i].getHeading())
                    .build();
            double pickUpDistance = isRed ? PICKUP_DISTANCE : -PICKUP_DISTANCE;
            Pose endPose = new Pose(pickupOrderPoses[i].getX() + pickUpDistance, pickupOrderPoses[i].getY(), pickupOrderPoses[i].getHeading());
            pickupEndPathChains[i] = follower.pathBuilder()
                    .addPath(new BezierLine(pickupOrderPoses[i], endPose))
                    .setLinearHeadingInterpolation(pickupOrderPoses[i].getHeading(), endPose.getHeading())
                    .build();
            shootPathChains[i] = follower.pathBuilder()
                    .addPath(new BezierLine(endPose, shootPose))
                    .setLinearHeadingInterpolation(endPose.getHeading(), shootPose.getHeading())
                    .build();
        }
        
        // Build path to end pose (from shoot pose)
        endPathChain = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(shootPreload);
            setPathState(pathState + 1);
            return;
        } else {
            int i = pathState - 1;
            switch (actionState) {
                case SHOOT:
//                    Log.i("AutoBase", "0. In SHOOT, isBusy: "+ follower.isBusy());
                    if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
//                        Log.i("AutoBase", "1. In SHOOT, isBusy: "+ follower.isBusy());
                        // We are at shoot pose.
                        // Ideally we started shooting early. Ensure it's on.
                        if(!robot.isShooting()) {
//                            Log.i("AutoBase", "2. In SHOOT, isBusy: "+ follower.isBusy());
                            robot.startShooting();
                        }
                        
                        // Wait for shot to complete (GreenApple handles the shot sequence)
                        // But GreenApple states (LAUNCH_BALL) happen automatically if robot.isShooting() is true.
                        // We just need to wait until we are empty?
                        // Or check if robot is 'Busy' shooting.
                        
                        // Wait until GreenApple says it's done shooting (e.g. going back to IDLE or COLLECT)
                        // Actually, GreenApple.update() handles "LAUNCH_BALL -> IDLE".
                        // So if we are in IDLE and isShooting() is true, it might be spin up.
                        // If we have balls, wait.
                        
                        if (!robot.getIndexer().hasBalls()) {
                             // Done shooting
                             robot.stopShooting();
                             // Move to next
                            if (i >= pickupOrderPathChains.length)  {
                                setActionState(ActionState.GO_TO_END);
                            } else {
                                follower.followPath(pickupOrderPathChains[i], .9, true);
                                setActionState(ActionState.GO_TO_PICKUP_POSE);
                            }
                        }
                    }
                    break;
                case GO_TO_PICKUP_POSE:
                    if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                        follower.followPath(pickupEndPathChains[i],0.5,  true);
                        setActionState(ActionState.PICKUP);
                    }
                    break;
                case PICKUP:
                    if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                        // Start moving to shoot
                        follower.followPath(shootPathChains[i], .9, true);
                        setActionState(ActionState.GO_TO_SHOOT_POSE);
                    }
                    break;
                case GO_TO_SHOOT_POSE:
                    if (actionTimer.getElapsedTimeSeconds() < INTAKE_TIME) {
                        robot.startIntake();
                    } else {
                        robot.stopIntake();

                    }
                    if (!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT) {
                        setActionState(ActionState.SHOOT);
                        setPathState(pathState + 1);
                    }
                    break;
                case GO_TO_END:
                    // Stop shooting when we enter this state
                    robot.stopShooting();
                    
                    // If follower is not busy, we haven't started the path yet - start it
                    if (!follower.isBusy()) {
                        follower.followPath(endPathChain, 1, true);
                    }
                    // If follower is busy, we're already following the path - just wait
                    // When path completes, follower.isBusy() will be false but we're done
                    break;
            }
        }
    }
}
