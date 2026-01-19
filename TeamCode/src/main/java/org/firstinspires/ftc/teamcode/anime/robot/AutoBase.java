package org.firstinspires.ftc.teamcode.anime.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class AutoBase extends OpMode {

    private static final double PATH_TIMEOUT = 2.0;
    private static final double PICKUP_DISTANCE = 30;

    private enum ActionState {
        SHOOT,
        GO_TO_PICKUP_POSE,
        PICKUP,
        GO_TO_SHOOT_POSE,
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

    protected int currentPattern = 0;

    protected Follower follower;
    protected Timer pathTimer, actionTimer, opmodeTimer;
    protected int pathState;
    protected ActionState actionState;
    protected Limelight limelight;
    protected GreenApple robot;

    public abstract void assignPosesToVariables();

    @Override
    public void init() {
        assignPosesToVariables();
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        this.robot = new GreenApple(hardwareMap, telemetry, true);
        this.limelight = robot.getLimelight();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
//        findPattern();
        robot.setFlyWheelShootingVelocity(shootingVelocity);
        robot.getIndexer().setAllBallsTrue();
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setActionState(ActionState.SHOOT);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        robot.update();
        PoseStorage.currentPose = follower.getPose();

        // Feedback to Driver Hub for debugging
        updateTelemetry();
    }


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
        telemetry.addData("April Tag Pattern", currentPattern);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Action State", actionState.name());
        telemetry.addData("Action Time", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Robot Pose", follower.getPose().toString());
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
        while (currentPattern == 0 || opmodeTimer.getElapsedTimeSeconds() < 5.0) {
            int aprilTagId = limelight.getAprilTagId();
            if (aprilTagId >= GPP_TAG_ID && aprilTagId <= PPG_TAG_ID) {
                currentPattern = aprilTagId;
            } else {
                idle();
            }
            updateTelemetry();
        }
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
                    .addPath(new BezierLine(prevPose, pickupOrderPoses[i]))
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
    }

    public void autonomousPathUpdate() {
        if (pathState == 0) {
            follower.followPath(shootPreload);
            setPathState(pathState + 1);
            return;
        } else {
            int i = pathState - 1;
            if (i >= pickupOrderPathChains.length) return;
            switch (actionState) {
                case SHOOT:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT) {
                        if(!robot.isShooting()) {
                            robot.startShooting();
                        } else {
                            boolean shouldMoveToPickup = !robot.isBusy();
                            if (shouldMoveToPickup) {
                                robot.stopShooting();
                                follower.followPath(pickupOrderPathChains[i], true);
                                setActionState(ActionState.GO_TO_PICKUP_POSE);
                            }
                        }
                    }
                    break;
                case GO_TO_PICKUP_POSE:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT) {
                        follower.followPath(pickupEndPathChains[i], true);
                        setActionState(ActionState.PICKUP);
                    }
                    break;
                case PICKUP:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT) {
                        follower.followPath(shootPathChains[i], true);
                        setActionState(ActionState.GO_TO_SHOOT_POSE);
                    }
                    break;
                case GO_TO_SHOOT_POSE:
                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT) {
                        setActionState(ActionState.SHOOT);
                        setPathState(pathState + 1);
                    }
                    break;
            }
        }
    }
}
