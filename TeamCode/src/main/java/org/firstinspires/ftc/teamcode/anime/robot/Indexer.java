package org.firstinspires.ftc.teamcode.anime.robot;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Indexer {

    private final CRServoImplEx indexerServo;
    private Telemetry telemetry;

    public Indexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.indexerServo = hardwareMap.get(CRServoImplEx.class, "index-servo");
        this.telemetry = telemetry;
    }

    public void start(double power, boolean slow) {
        if (slow) {
            this.indexerServo.setPower(power * 0.1);
        } else {
            this.indexerServo.setPower(power * 0.5);
        }
    }
}
