package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.anime.robot.Indexer;
import org.firstinspires.ftc.teamcode.anime.robot.Intake;
import org.firstinspires.ftc.teamcode.anime.robot.Lift;
import org.firstinspires.ftc.teamcode.anime.robot.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name="Anime: Teleop", group="Anime")
public class AnimeTeleop extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.2;
    private double slowModeIncrement = 0.2;
    private boolean automatedDrive = false;
//    private Supplier<PathChain> pathChain;

    private Shooter shooter;
    private Lift lift;
    private Intake intake;
    private Indexer indexer;
    private boolean indexerSlow = false;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72)); // TODO: Get latest pose from Auto End.
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        shooter = new Shooter(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);

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

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
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
            slowMode = !slowMode;
        }
        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += slowModeIncrement;
            if (slowModeMultiplier > 1) {
                slowModeMultiplier = 1.0;
            }
        }
        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier -= slowModeIncrement;
            if (slowModeMultiplier < slowModeIncrement) {
                slowModeMultiplier = slowModeIncrement;
            }
        }

        shooter.start(gamepad2.right_trigger);
        lift.start(gamepad2.left_trigger);
        intake.start(-gamepad2.right_stick_y);
        if(gamepad2.xWasPressed()) {
            indexerSlow = !indexerSlow;
        }
        indexer.start(gamepad2.left_stick_x, indexerSlow);

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.debug("slow mode:" + this.slowMode);
        telemetryM.debug("slow mode multiplier:" + this.slowModeMultiplier);
        telemetryM.debug("indexer slow mode:" + this.indexerSlow);
        telemetryM.debug("shooter velocity:" + this.shooter.getVelocity());
        telemetryM.debug("shooter power:" + this.shooter.getPower());
        telemetryM.debug("shooter velocity:" + this.shooter.getVelocity());
        telemetryM.debug("shooter power:" + this.shooter.getPower());
        telemetryM.debug("shooter power %:" + (this.shooter.getPower() * 100) + "%");
        telemetryM.debug("shooter velocity (ticks/sec):" + this.shooter.getVelocity());
        //On GoBilda PPR is 28 and is a quadrature encoder so 28*4 = 112 ticks/revolution
        double TICKS_PER_REVOLUTION = 112.0;
        double rpm = (this.shooter.getVelocity() / TICKS_PER_REVOLUTION) * 60;
        telemetryM.debug("shooter RPM:" + rpm);
        telemetryM.debug((rpm * 0.67));
        telemetryM.update(telemetry);
    }
}
