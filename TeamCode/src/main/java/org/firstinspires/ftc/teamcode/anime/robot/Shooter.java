package org.firstinspires.ftc.teamcode.anime.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    public static final double MAX_VELOCITY = 2200;
    public static final double GREEN_COLOR_VALUE = 0.5;
    private final DcMotorEx shooterMotor;
    private final Servo rgbLight;
    private Telemetry telemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        this.rgbLight = hardwareMap.get(Servo.class, "rgb-light");
        this.telemetry = telemetry;
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

//    public void start(double power) {
//        double cappedPower = Math.min(power, 0.8);
//        Utils.setSafePower(this.shooterMotor, cappedPower);
//    }

    public void setVelocity(double velocity, boolean showLight) {
        this.shooterMotor.setVelocity(velocity);
        double currentVelocity = getVelocity();
        if(showLight && currentVelocity > 100) {
            if(velocity == 0) {
                this.rgbLight.setPosition((getVelocity() / MAX_VELOCITY) * GREEN_COLOR_VALUE);
            } else {
                this.rgbLight.setPosition((getVelocity() / velocity) * GREEN_COLOR_VALUE);
            }
        }
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

    public boolean isAtVelocity(double flyWheelVelocity) {
        return Math.abs(getVelocity() - flyWheelVelocity) < 100;
    }
}
