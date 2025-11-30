package org.firstinspires.ftc.teamcode.anime.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private final DcMotorEx intakeMotor;
    private Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        this.telemetry = telemetry;
    }

    public void start(double power) {
        Utils.setSafePower(this.intakeMotor, power);
    }
}
