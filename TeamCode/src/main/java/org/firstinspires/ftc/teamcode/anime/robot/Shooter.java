package org.firstinspires.ftc.teamcode.anime.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private final DcMotorEx shooterMotor;
    private Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        this.telemetry = telemetry;
    }

    public void start(double power) {
        Utils.setSafePower(this.shooterMotor, power);
    }
}
