package org.firstinspires.ftc.teamcode.anime.opmodes;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.anime.robot.GreenApple;
import org.firstinspires.ftc.teamcode.anime.robot.Indexer;
import org.firstinspires.ftc.teamcode.anime.robot.Intake;
import org.firstinspires.ftc.teamcode.anime.robot.Lift;
import org.firstinspires.ftc.teamcode.anime.robot.Limelight;
import org.firstinspires.ftc.teamcode.anime.robot.PoseStorage;
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


    private GreenApple robot;
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
    private boolean gamepad2BWasPressed = false;
    private final double ALIGN_KP = 0.03;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new GreenApple(hardwareMap, telemetry, false);


        shooter = robot.getShooter();
        lift = robot.getLift();
        intake = robot.getIntake();
        indexer = robot.getIndexer();
        limelight = robot.getLimelight();

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
            double drivePowerX = -gamepad1.left_stick_y;
            double drivePowerY = -gamepad1.left_stick_x;
            double turnPower = -gamepad1.right_stick_x;

            if (driveInSlowMode) {
                drivePowerX *= driveSlowModeMultiplier;
                drivePowerY *= driveSlowModeMultiplier;
                turnPower *= driveSlowModeMultiplier;
            } else {
                turnPower *= 0.5;
            }

            // Auto-align to AprilTag
            if (gamepad1.a) {
                int tagId = limelight.getAprilTagId();
                if (tagId != -1) {
                    Double tx = limelight.getHorizontalOffset(tagId);
                    if (tx != null) {
                        turnPower = -tx * ALIGN_KP;
                        telemetryM.debug("Aligning to Tag " + tagId + " tx: " + tx);
                    }
                }
            }

            follower.setTeleOpDrive(drivePowerX, drivePowerY, turnPower, true);
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

        // Endgame lift
        if (gamepad1.dpad_up) {
            lift.startServos(1.0);
        } else if (gamepad1.dpad_down) {
            lift.stopServos();
        }

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

        if(gamepad2.right_trigger >= 0.2) {
            calculatedVelocity = limelight.getVelocity();
//            Log.i("AnimeTeleop", "calculatedVelocity: " + calculatedVelocity);
            if (calculatedVelocity != -1) {
                // Target detected - use calculated velocity from limelight
                useAutoVelocity = true;
                shooter.setVelocity(calculatedVelocity, true);
            } else {
                // Target not detected - don't rev the shooter
                shooter.setVelocity(Shooter.MAX_VELOCITY * gamepad2.right_trigger, true);
            }
        } else {
            shooter.setVelocity(0, true);
        }

        if (indexer.isReadyToShoot()) {
            indexer.updateShootingPos();
            lift.start(1.0);
        } else {
            lift.start(gamepad2.left_trigger);
        }
        intake.start(-gamepad2.right_stick_y);
        if(gamepad2.xWasPressed()) {
            indexerSlow = !indexerSlow;
        }

        if(gamepad2BWasPressed != gamepad2.b && gamepad2.b) {
            indexer.alignToBestBall();
        }
        gamepad2BWasPressed = gamepad2.b;

        boolean isShooting = gamepad2.right_trigger >= 0.2 && gamepad2.left_trigger > 0.2;
        boolean isIntaking = Math.abs(gamepad2.right_stick_y) > 0.2;
        if (isShooting) {
//            Log.i("AnimeTeleop", "Shooting Mode");
            indexer.updateShootingPos();
            if(dpadRightWasPressed != gamepad2.dpad_right && gamepad2.dpad_right) {
                indexer.goToNextShootAngle();
            } else if(dpadLeftWasPressed != gamepad2.dpad_left && gamepad2.dpad_left) {
                indexer.goToPrevShootAngle();
            } else {
                indexer.start(gamepad2.left_stick_x, indexerSlow);
            }
        } else if (isIntaking) {
//            Log.i("AnimeTeleop", "Intake Mode");
            indexer.emptyPatternQueue();
            indexer.updateIntakePos();
            if (indexer.hasAllBalls()) {
                indexer.setLight(1.0);
            } else {
                indexer.setLight(0.0);
                if (indexer.hasBallInIntakePosition()) {
                    indexer.goToNextEmptyIntakeAngle();
                } else if(dpadRightWasPressed != gamepad2.dpad_right && gamepad2.dpad_right) {
                    indexer.goToNextIntakeAngle();
                } else if(dpadLeftWasPressed != gamepad2.dpad_left && gamepad2.dpad_left) {
                    indexer.goToPrevIntakeAngle();
                } else {
                    indexer.start(gamepad2.left_stick_x, indexerSlow);
                }
            }
        } else {
//            Log.i("AnimeTeleop", "No Mode");
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
