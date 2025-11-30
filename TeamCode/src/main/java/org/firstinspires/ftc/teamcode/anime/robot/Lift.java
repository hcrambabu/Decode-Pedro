package org.firstinspires.ftc.teamcode.anime.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private final DcMotorEx liftMotor;
    private Telemetry telemetry;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    public void start(double power) {
        Utils.setSafePower(this.liftMotor, power);
    }
}
