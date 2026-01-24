package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.anime.robot.Indexer;
import org.firstinspires.ftc.teamcode.anime.robot.Intake;
import org.firstinspires.ftc.teamcode.anime.robot.Lift;
import org.firstinspires.ftc.teamcode.anime.robot.Limelight;
import org.firstinspires.ftc.teamcode.anime.robot.Shooter;

import java.util.List;

@Configurable
@TeleOp(name="Anime: Reset", group="Anime")
public class ResetEncoders  extends OpMode {

    private Shooter shooter;
    private Lift lift;
    private Intake intake;
    private Indexer indexer;
    private Limelight limelight;

    private int limelightPipeline = 9;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry, false);
        limelight = new Limelight(hardwareMap, limelightPipeline, telemetry);

        shooter.resetEncoder();
        indexer.resetEncoder();
        hardwareMap.get(DcMotorEx.class, "rf").setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotorEx.class, "rb").setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotorEx.class, "lf").setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotorEx.class, "lb").setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        if(gamepad1.a || gamepad2.a) {
            limelightPipeline = 8;
            limelight.changePipeline(limelightPipeline);
        } else if(gamepad1.b || gamepad2.b) {
            limelightPipeline = 9;
            limelight.changePipeline(limelightPipeline);
        }

        telemetry.addData("Shooter light Detected: ", indexer.getShootColorAndDistanceSensor().getLightDetected());
        telemetry.addData("Intake  light Detected: ", indexer.getIntakeColorAndDistanceSensor().getLightDetected());
        List<Integer> tags = limelight.getAllAprilTagId();
        telemetry.addData("AprilTags Pipeline: "+ limelightPipeline +", Detected: ", tags.toString());
        telemetry.update();
    }
}
