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
        double cappedPower = Math.min(power, 0.8);
        Utils.setSafePower(this.shooterMotor, cappedPower);
    }

    public double getVelocity() {
        return this.shooterMotor.getVelocity();
    }

    public double getPower() {
        return this.shooterMotor.getPower();
    }
}
