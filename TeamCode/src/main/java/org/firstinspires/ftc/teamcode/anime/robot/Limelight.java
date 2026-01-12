package org.firstinspires.ftc.teamcode.anime.robot;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Limelight {

    private Limelight3A limelight;
    private IMU imu;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9);
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public double getDistance(double ta) {
        double a = 2742.663;
        double b = -1.535693;

        if (ta <= 0) return -1;

        return Math.pow(ta / a, 1.0 / b)/2.54;
    }

    public double getVelocityFromDistance(double distanceCm) {
        double x = distanceCm;

//        double y = 4084.3
//                - (125.4496 * x)
//                + (1.763109 * x * x)
//                - (0.007353914 * x * x * x);

        double y = (4084.3
                - (125.4496 * x)
                + (1.763109 * x * x)
                - (0.007353914 * x * x * x)) +25 ;

        return Math.abs(y);
    }

    public double getVelocity() {
        try {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llresult = limelight.getLatestResult();

            Log.i("Limelight", "llresult: " + llresult);
            if (llresult != null && llresult.isValid()) {
                double ta = llresult.getTa();
                Log.i("Limelight", "ta: " + ta);
                if (ta > 0) {
                    double distanceCm = getDistance(ta);
                    Log.i("Limelight", "distanceCm: " + distanceCm);
                    if (distanceCm >= 0) {
                        double v = getVelocityFromDistance(distanceCm);
                        Log.i("Limelight", "v: " + v);
                        return v;
                    }
                }
            }
        } catch (Exception e) {
            Log.i("Limelight", "Exception in getVelocity", e);
        }
        return -1;
    }
}
