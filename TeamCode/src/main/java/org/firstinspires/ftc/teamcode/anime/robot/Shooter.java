package org.firstinspires.ftc.teamcode.anime.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    public static final double MAX_VELOCITY = 5000;
    private final DcMotorEx shooterMotor;
    private Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        this.telemetry = telemetry;
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

//    public void start(double power) {
//        double cappedPower = Math.min(power, 0.8);
//        Utils.setSafePower(this.shooterMotor, cappedPower);
//    }

    public void setVelocity(double velocity) {
        this.shooterMotor.setVelocity(velocity);
    }

    public double getVelocity() {
        return this.shooterMotor.getVelocity();
    }

    public double getPower() {
        return this.shooterMotor.getPower();
    }

    public void resetEncoder() {
        this.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
