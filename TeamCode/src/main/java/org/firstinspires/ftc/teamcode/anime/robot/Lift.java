package org.firstinspires.ftc.teamcode.anime.robot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private final DcMotorEx liftMotor;

    private final ServoImplEx rServo;
    private final ServoImplEx lServo;
    private Telemetry telemetry;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        rServo = hardwareMap.get(ServoImplEx.class, "r-lift");
        lServo = hardwareMap.get(ServoImplEx.class, "l-lift");
        rServo.setPwmRange(new PwmControl.PwmRange(900, 2100));
        lServo.setPwmRange(new PwmControl.PwmRange(900, 2100));
        rServo.setPosition(0.0);
        lServo.setPosition(0.0);
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    public void start(double power) {
        Utils.setSafePower(this.liftMotor, power);
    }

    public void stop() {
        this.liftMotor.setPower(0);
    }

    public void startServos(double position) {
        rServo.setPosition(position);
        lServo.setPosition(position);
    }

    public void stopServos() {
        rServo.setPosition(0.0);
        lServo.setPosition(0.0);
    }
}
