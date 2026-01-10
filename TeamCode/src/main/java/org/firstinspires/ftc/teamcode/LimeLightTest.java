package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class LimeLightTest extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9); //<-- TODO

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()){
            Pose3D botPose = llresult.getBotpose();
            double distanceCm = getDistance(llresult.getTa());
            double distanceIn = distanceCm / 2.54;
            double shooterVelocity = getVelocityFromDistance(distanceCm);
            
            telemetry.addData("Distance (cm)", distanceCm);
            telemetry.addData("Distance (in)", distanceIn);
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Ta", llresult.getTa());
            telemetry.addData("Shooter Velocity", shooterVelocity);
        }

    }

    public double getDistance(double ta){
//        double a = 2290.861;
//        double b = -1.485562;
//        double a = 2125.609;
//        double b = -1.475733;
//        double a = 2423.512;
//        double b = -1.506672;
        double a = 2742.663;
        double b = -1.535693;

        // guard against bad readings
        if (ta <= 0) return Double.POSITIVE_INFINITY;

        return Math.pow(ta / a, 1.0 / b);

    }

    public double getVelocityFromDistance(double distanceCm){
        double x = distanceCm;
        
        double y = 4084.3 
                - (125.4496 * x) 
                + (1.763109 * x * x) 
                - (0.007353914 * x * x * x);
        
        if (y < 0 || Double.isNaN(y) || Double.isInfinite(y)) {
            return 0;
        }
        
        return y;
    }

}
