package org.firstinspires.ftc.teamcode.anime.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.anime.robot.Indexer;
import org.firstinspires.ftc.teamcode.anime.robot.Intake;
import org.firstinspires.ftc.teamcode.anime.robot.Lift;
import org.firstinspires.ftc.teamcode.anime.robot.Shooter;

@Configurable
@TeleOp(name="Anime: Reset", group="Anime")
public class ResetEncoders  extends OpMode {

    private Shooter shooter;
    private Lift lift;
    private Intake intake;
    private Indexer indexer;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry, false);

        shooter.resetEncoder();
        indexer.resetEncoder();
        hardwareMap.get(DcMotorEx.class, "rf").setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotorEx.class, "rb").setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotorEx.class, "lf").setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotorEx.class, "lb").setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

    }
}
