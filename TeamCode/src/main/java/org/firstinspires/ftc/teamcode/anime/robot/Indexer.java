package org.firstinspires.ftc.teamcode.anime.robot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Indexer {
    private static final double TICKS_PER_REV = 751.8;
    private static final double TIME_OUT = 1.0;
    private static final double[] INTAKE_ANGLES = {60, 180, 300};
    private static final double MANUAL_INTERVENTION_THRESHOLD = 45.0;
    private static final double SPEED_MULTIPLIER = 0.6;
    private static final double SLOW_SPEED_MULTIPLIER = 0.1;

    private final DcMotorEx indexerMotor;
    private final ElapsedTime timer;
    private Telemetry telemetry;
    private int targetIndex = 0;

    public Indexer(HardwareMap hardwareMap, Telemetry telemetry, boolean restMotorPosition) {
        this.indexerMotor = hardwareMap.get(DcMotorEx.class, "indexer");
        this.indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
        timer = new ElapsedTime();
        if (restMotorPosition) {
            indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            syncToPhysicalPosition();
        }
    }

    public void start(double power, boolean slow) {
        if (isBusy(power != 0)) {
            return;
        }
        if (slow) {
            this.indexerMotor.setPower(power * SLOW_SPEED_MULTIPLIER);
        } else {
            this.indexerMotor.setPower(power * SPEED_MULTIPLIER);
        }
    }
    public void stop() {
        this.indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.indexerMotor.setPower(0);
    }

    public boolean isBusy(boolean cancelTheCurrentJob) {
        if (cancelTheCurrentJob) {
            stop();
            return false;
        }
        if (this.indexerMotor.isBusy() && this.timer.seconds() < TIME_OUT) {
            return true;
        }
        if (this.indexerMotor.isBusy()) {
            stop();
        }
        return false;
    }

    public int getCurrentPosition() {
        return this.indexerMotor.getCurrentPosition();
    }

    /**
     * Calculates the current angle based on encoder ticks.
     * Assumes start position was 0.
     */
    public double getAngle() {
        return (indexerMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    /**
     * Explicitly sets the internal state (targetIndex) to match the current physical motor position.
     * Call this in init() or if you suspect the robot was moved manually while disabled.
     */
    public void syncToPhysicalPosition() {
        this.targetIndex = getClosestIndex(getAngle());
    }

    private int getClosestIndex(double currentAngle) {
        int closestIndex = 0;
        double minDifference = Double.MAX_VALUE;

        // Normalize current angle to 0-360
        double normalizedAngle = currentAngle % 360;
        if (normalizedAngle < 0) normalizedAngle += 360;

        for (int i = 0; i < INTAKE_ANGLES.length; i++) {
            double diff = Math.abs(normalizedAngle - INTAKE_ANGLES[i]);
            double diffWrapped = 360 - diff;
            double shortestDiff = Math.min(diff, diffWrapped);

            if (shortestDiff < minDifference) {
                minDifference = shortestDiff;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    /**
     * Sets the target angle.
     * Calculates the shortest path to the target angle (handling wrap-around).
     */
    public void setTargetAngle(double targetAngleDegrees, double power) {
        double currentAngle = getAngle();

        // Calculate raw difference
        double difference = targetAngleDegrees - currentAngle;

        // Robust Normalize to [-180, 180]
        // This handles cases where currentAngle is negative (e.g. -290)
        // or effectively > 360 away.
        double delta = (difference % 360 + 360) % 360; // Normalize to [0, 360)

        if (delta > 180) {
            delta -= 360; // Shift to [-180, 180]
        }

        // Calculate target ticks relative to CURRENT TOTAL position
        int currentPosTicks = indexerMotor.getCurrentPosition();
        int deltaTicks = (int) ((delta / 360.0) * TICKS_PER_REV);
        int targetTicks = currentPosTicks + deltaTicks;

        Log.i("Indexer", "Setting target angle to " + targetAngleDegrees + " degrees (delta: " + delta + ", target ticks: " + targetTicks + ")");

        indexerMotor.setTargetPosition(targetTicks);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexerMotor.setPower(power);
        timer.reset();
    }

    public void goToNextIntakeAngle() {
        double currentAngle = getAngle();

        // Normalize angle for comparison
        double normalizedAngle = currentAngle % 360;
        if (normalizedAngle < 0) normalizedAngle += 360;

        // Check for manual intervention
        // Calculate distance between current physical angle and what we THINK is the target
        double errorFromState = Math.abs(normalizedAngle - INTAKE_ANGLES[targetIndex]);
        if (errorFromState > 180) errorFromState = 360 - errorFromState;

        // If error is large, the user moved the motor manually. Sync state first.
        if (errorFromState > MANUAL_INTERVENTION_THRESHOLD) {
            targetIndex = getClosestIndex(currentAngle);
        }

        // Increment index
        targetIndex = (targetIndex + 1) % INTAKE_ANGLES.length;

        setTargetAngle(INTAKE_ANGLES[targetIndex], SPEED_MULTIPLIER);
    }

    public void goToPrevIntakeAngle() {
        double currentAngle = getAngle();

        // Normalize angle for comparison
        double normalizedAngle = currentAngle % 360;
        if (normalizedAngle < 0) normalizedAngle += 360;

        // Check for manual intervention
        double errorFromState = Math.abs(normalizedAngle - INTAKE_ANGLES[targetIndex]);
        if (errorFromState > 180) errorFromState = 360 - errorFromState;

        if (errorFromState > MANUAL_INTERVENTION_THRESHOLD) {
            targetIndex = getClosestIndex(currentAngle);
        }

        // Decrement index
        targetIndex = (targetIndex - 1 + INTAKE_ANGLES.length) % INTAKE_ANGLES.length;

        setTargetAngle(INTAKE_ANGLES[targetIndex], SPEED_MULTIPLIER);
    }
}
