package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.anime.robot.Indexer;
import org.firstinspires.ftc.teamcode.anime.robot.Intake;
import org.firstinspires.ftc.teamcode.anime.robot.Lift;
import org.firstinspires.ftc.teamcode.anime.robot.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name="Anime: Auto Center", group="Anime")
public class AnimeAutoCenter extends LinearOpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    private Shooter shooter;
    private Lift lift;
    private Intake intake;
    private Indexer indexer;
    
    private ElapsedTime runtime;
    private ElapsedTime stateTimer;
    
    // Autonomous state machine
    public enum AutoState {
        INIT,
        DRIVE_TO_SHOOT_POS,
        SPIN_UP_SHOOTER,
        SHOOT_PRELOAD,
        DRIVE_TO_PARK,
        PARK,
        DONE
    }
    
    private AutoState currentState = AutoState.INIT;
    
    // Starting pose - adjust based on actual starting position
    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    // Shooting pose - adjust based on goal position
    private final Pose shootPose = new Pose(56, 86, Math.toRadians(45));
    // Parking pose - adjust based on parking zone
    private final Pose parkPose = new Pose(72, 90, Math.toRadians(0));
    
    private PathChain driveToShootPath;
    private PathChain driveToParkPath;
    
    // Shooter velocity for autonomous (adjust as needed)
    private final double SHOOTER_VELOCITY = 1650;
    private final double SHOOTER_SPIN_UP_TIME = 2.0; // seconds
    private final double SHOOT_TIME = 1.0; // seconds
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        
        shooter = new Shooter(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry, false);
        
        runtime = new ElapsedTime();
        stateTimer = new ElapsedTime();
        
        // Build paths
        buildPaths();
        
        telemetry.addData("Status", "Initialized - Ready to start");
        telemetry.update();
        
        waitForStart();
        
        runtime.reset();
        stateTimer.reset();
        currentState = AutoState.DRIVE_TO_SHOOT_POS;
        
        // Main autonomous loop - runs for 30 seconds max
        while (opModeIsActive() && runtime.seconds() < 30.0) {
            follower.update();
            stateMachine();
            updateTelemetry();
        }
        
        // Stop all mechanisms at end of autonomous
        stopAllMechanisms();
    }
    
    private void buildPaths() {
        // Path from start position to shooting position
        driveToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        
        // Path from shooting position to parking position
        driveToParkPath = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }
    
    private void stateMachine() {
        switch (currentState) {
            case DRIVE_TO_SHOOT_POS:
                if (!follower.isBusy()) {
                    // Start driving to shoot position
                    follower.followPath(driveToShootPath);
                }
                // Check if path is complete
                if (follower.isBusy() && follower.atParametricEnd()) {
                    currentState = AutoState.SPIN_UP_SHOOTER;
                    stateTimer.reset();
                }
                break;
                
            case SPIN_UP_SHOOTER:
                // Spin up shooter to target velocity
                shooter.setVelocity(SHOOTER_VELOCITY);
                
                // Wait for shooter to reach speed
                if (stateTimer.seconds() >= SHOOTER_SPIN_UP_TIME) {
                    currentState = AutoState.SHOOT_PRELOAD;
                    stateTimer.reset();
                }
                break;
                
            case SHOOT_PRELOAD:
                // Keep shooter at speed
                shooter.setVelocity(SHOOTER_VELOCITY);
                
                // Index the preloaded artifact to shoot
                // Adjust indexer position as needed for shooting
                indexer.start(1.0, false); // Feed artifact to shooter
                
                // Wait for shot to complete
                if (stateTimer.seconds() >= SHOOT_TIME) {
                    indexer.stop();
                    currentState = AutoState.DRIVE_TO_PARK;
                    stateTimer.reset();
                }
                break;
                
            case DRIVE_TO_PARK:
                // Stop shooter to save power
                shooter.setVelocity(0);
                
                if (!follower.isBusy()) {
                    // Start driving to park position
                    follower.followPath(driveToParkPath);
                }
                
                // Check if path is complete
                if (follower.isBusy() && follower.atParametricEnd()) {
                    currentState = AutoState.PARK;
                    stateTimer.reset();
                }
                break;
                
            case PARK:
                // Robot is parked, wait for autonomous to end
                currentState = AutoState.DONE;
                break;
                
            case DONE:
                // Autonomous complete
                break;
        }
    }
    
    private void stopAllMechanisms() {
        shooter.setVelocity(0);
        lift.start(0);
        intake.start(0);
        indexer.stop();
    }
    
    private void updateTelemetry() {
        telemetryM.debug("State: " + currentState.toString());
        telemetryM.debug("Runtime: " + String.format("%.2f", runtime.seconds()));
        telemetryM.debug("X: " + String.format("%.2f", follower.getPose().getX()));
        telemetryM.debug("Y: " + String.format("%.2f", follower.getPose().getY()));
        telemetryM.debug("Heading: " + String.format("%.2f", Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("Shooter Velocity: " + String.format("%.2f", shooter.getVelocity()));
        telemetryM.update(telemetry);
    }
}

