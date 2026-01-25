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

    private static final double ACTION_TIMEOUT = 2.0;
    private static final double PICKUP_DISTANCE = 45;
    private static final double INTAKE_TIME = 3.0;
    private static final double ALIGN_KP = 0.03;
    private static final double ALIGN_THRESHOLD = 1.0; // degrees - only align if offset is greater than this
    private static final double SHOOTING_MAX_TIME = 8.0; // Maximum time to spend in SHOOT state before forcing transition
    private static final double AUTO_TIME_LIMIT = 30.0; // Total autonomous time limit
    private static final double PARK_TIME_REMAINING = 3.0; // Start parking when this much time remains
    private boolean isAlignedForShooting = false;
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
    protected double alignmentOffset = 0.0; // Offset in degrees for alignment (negative = left, positive = right)

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
    private int shotsFired = 0; // Track number of shots fired at current shooting position
    private boolean wasShootingPositionEmpty = false; // Track previous state of shooting position

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
        // Check if we need to force park in the last 3 seconds
        double timeRemaining = AUTO_TIME_LIMIT - opmodeTimer.getElapsedTimeSeconds();
        if (timeRemaining <= PARK_TIME_REMAINING && actionState != ActionState.GO_TO_END) {
            // Force transition to end position
            robot.stopShooting();
            robot.stopIntake();
            isAlignedForShooting = false;
            setActionState(ActionState.GO_TO_END);
        }
        
        if (pathState == 0) {
            follower.followPath(shootPreload);
            setPathState(pathState + 1);
            return;
        } else {
            int i = pathState - 1;
            switch (actionState) {
                case SHOOT:
                    // Once we reach the shoot pose, switch to teleop drive for rotation control
                    if (!follower.isBusy() && !isAlignedForShooting) {
                        follower.startTeleopDrive();
                        isAlignedForShooting = true; // Mark that we've switched to teleop mode
                        shotsFired = 0; // Reset shot counter when entering shoot position
                        wasShootingPositionEmpty = false; // Reset tracking
                    }
                    
                    // Always run alignment and shooting logic (not dependent on follower state)
                    // Auto-align to goal AprilTag - continuously adjust heading
                    int goalTagId = isRed ? RED_GOAL_TAG_ID : BLUE_GOAL_TAG_ID;
                    Double tx = limelight.getHorizontalOffset(goalTagId);
                    if (tx != null) {
                        // Apply alignment offset (for bottom autonomous: -5 degrees = 5 degrees to the left)
                        double adjustedTx = tx + alignmentOffset;
                        if (Math.abs(adjustedTx) > ALIGN_THRESHOLD) {
                            double turnPower = -adjustedTx * ALIGN_KP;
                            // Clamp turn power for smoother rotation
                            turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
                            follower.setTeleOpDrive(0, 0, turnPower, true);
                        } else {
                            // Aligned - stop rotation
                            follower.setTeleOpDrive(0, 0, 0, true);
                        }
                    }
                    
                    // Start shooting once we're at pose (follower not busy) or after timeout
                    // This ensures shooting starts even if alignment fails
                    if ((!follower.isBusy() || actionTimer.getElapsedTimeSeconds() > ACTION_TIMEOUT)) {
                        if (!robot.isShooting()) {
                            robot.startShooting();
                        }
                    }
                    
                    // Track shots by detecting when shooting position becomes empty (ball was shot)
                    // This detects the transition from having a ball to empty, which indicates a shot
                    boolean isShootingPositionEmpty = !robot.getIndexer().hasBallInShootingPosition();
                    if (!wasShootingPositionEmpty && isShootingPositionEmpty && robot.isShooting()) {
                        // Ball was shot - increment counter (only if we were shooting)
                        shotsFired++;
                    }
                    wasShootingPositionEmpty = isShootingPositionEmpty;
                    
                    // Check if we've shot 3 times or enough time has passed for 3 shots
                    double elapsedTime = actionTimer.getElapsedTimeSeconds();
                    double timePerShot = 2.0; // Estimated time per shot (spin up + align + launch)
                    double minimumTimeFor3Shots = timePerShot * 3; // Minimum time needed for 3 shots
                    boolean enoughTimeFor3Shots = elapsedTime >= minimumTimeFor3Shots;
                    boolean forceTransition = elapsedTime > SHOOTING_MAX_TIME; // Safety timeout
                    
                    // Move after 3 shots are detected, OR after enough time has passed (assume 3 shots completed)
                    if (shotsFired >= 3 || (enoughTimeFor3Shots && shotsFired >= 2)) {
                        // Done shooting 3 balls - move immediately
                        robot.stopShooting();
                        isAlignedForShooting = false; // Reset for next time
                        shotsFired = 0; // Reset counter
                        // Move to next
                        if (i >= pickupOrderPathChains.length) {
                            setActionState(ActionState.GO_TO_END);
                        } else {
                            follower.followPath(pickupOrderPathChains[i], .9, true);
                            setActionState(ActionState.GO_TO_PICKUP_POSE);
                        }
                    } else if (forceTransition) {
                        // Safety: force transition if stuck too long (but still try to stop shooting)
                        robot.stopShooting();
                        isAlignedForShooting = false;
                        shotsFired = 0; // Reset counter
                        if (i >= pickupOrderPathChains.length) {
                            setActionState(ActionState.GO_TO_END);
                        } else {
                            follower.followPath(pickupOrderPathChains[i], .9, true);
                            setActionState(ActionState.GO_TO_PICKUP_POSE);
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
                        follower.followPath(shootPathChains[i], 1, true);
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
