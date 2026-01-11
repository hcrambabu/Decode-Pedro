package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.anime.robot.Indexer;
import org.firstinspires.ftc.teamcode.anime.robot.Intake;
import org.firstinspires.ftc.teamcode.anime.robot.Lift;
import org.firstinspires.ftc.teamcode.anime.robot.Limelight;
import org.firstinspires.ftc.teamcode.anime.robot.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name="Anime: Teleop", group="Anime")
public class AnimeTeleop extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    private boolean driveInSlowMode = false;
    private double driveSlowModeMultiplier = 0.2;
    private double slowModeIncrement = 0.2;
    private boolean automatedDrive = false;
//    private Supplier<PathChain> pathChain;

    private Shooter shooter;
    private Lift lift;
    private Intake intake;
    private Indexer indexer;

    private Limelight limelight;
    private boolean indexerSlow = false;

    private int shooterVelocityPreset = 3;
    private double[] shooterVelocityPresets = {1000, 2000, 3000, 4000, 5000, 5000};
    private boolean dpadLeftWasPressed = false;
    private boolean dpadRightWasPressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72)); // TODO: Get latest pose from Auto End.
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = new Shooter(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry, false);
        limelight = new Limelight(hardwareMap, telemetry);

//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Init Done - Press Start");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }
    private void handleShooterPresetSelection() {
        if (gamepad2.leftBumperWasPressed()) {
            shooterVelocityPreset = (shooterVelocityPreset - 1 + shooterVelocityPresets.length) % shooterVelocityPresets.length;
        } else if (gamepad2.rightBumperWasPressed()) {
            shooterVelocityPreset = (shooterVelocityPreset + 1) % shooterVelocityPresets.length;
        }
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!driveInSlowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * driveSlowModeMultiplier,
                    -gamepad1.left_stick_x * driveSlowModeMultiplier,
                    -gamepad1.right_stick_x * driveSlowModeMultiplier,
                    true // Robot Centric
            );
        }
//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            driveInSlowMode = !driveInSlowMode;
        }
        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            driveSlowModeMultiplier += slowModeIncrement;
            if (driveSlowModeMultiplier > 1) {
                driveSlowModeMultiplier = 1.0;
            }
        }
        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            driveSlowModeMultiplier -= slowModeIncrement;
            if (driveSlowModeMultiplier < slowModeIncrement) {
                driveSlowModeMultiplier = slowModeIncrement;
            }
        }

        handleShooterPresetSelection();

        boolean useAutoVelocity = false;
        double calculatedVelocity = 0.0f;
        if(gamepad2.right_trigger >= 0.5) {
            calculatedVelocity = limelight.getVelocity();
            if (calculatedVelocity >= 0) {
                useAutoVelocity = true;
                shooter.setVelocity(calculatedVelocity);
            } else {
                // TODO: manual mode?
            }
        } else {
            shooter.setVelocity(0);
        }

        lift.start(gamepad2.left_trigger);
        intake.start(-gamepad2.right_stick_y);
        if(gamepad2.xWasPressed()) {
            indexerSlow = !indexerSlow;
        }

        boolean isShooting = gamepad2.right_trigger >= 0.5 && gamepad2.left_trigger > 0 && (gamepad2.dpad_right || gamepad2.dpad_left);
        
        if (isShooting) {
            indexer.forceFeed(1.0);
        } else {
            if(dpadRightWasPressed != gamepad2.dpad_right && gamepad2.dpad_right) {
                indexer.goToNextIntakeAngle();
            } else if(dpadLeftWasPressed != gamepad2.dpad_left && gamepad2.dpad_left) {
                indexer.goToPrevIntakeAngle();
            } else {
                indexer.start(gamepad2.left_stick_x, indexerSlow);
            }
        }
        dpadRightWasPressed = gamepad2.dpad_right;
        dpadLeftWasPressed = gamepad2.dpad_left;


        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.debug("drive in slow mode:" + this.driveInSlowMode);
        telemetryM.debug("indexer slow mode:" + this.indexerSlow);
        telemetryM.debug("indexer angle:" + this.indexer.getAngle());
        telemetryM.debug("shooter velocity:" + this.shooter.getVelocity());
        if (useAutoVelocity) {
            telemetryM.debug("shooter mode: AUTO (Limelight)");
            telemetryM.debug("calculated velocity:" + calculatedVelocity);
        } else {
            telemetryM.debug("shooter mode: MANUAL");
            telemetryM.debug("shooter preset:" + shooterVelocityPreset + " target Velocity:" + shooterVelocityPresets[shooterVelocityPreset]);
        }
        telemetryM.debug("shooter power:" + this.shooter.getPower());
        telemetryM.debug("indexer front distance:" + this.indexer.getFrontDistance());
        telemetryM.debug("indexer back distance:" + this.indexer.getBackDistance());
        telemetryM.update(telemetry);
    }
}
