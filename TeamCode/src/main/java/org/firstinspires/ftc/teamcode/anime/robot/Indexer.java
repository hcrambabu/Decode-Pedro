package org.firstinspires.ftc.teamcode.anime.robot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Indexer {
    private static final double TICKS_PER_REV = 751.8;
    private static final double TIME_OUT = 1.0;
    private static final double[] INTAKE_ANGLES = {60, 180, 300};
    private static final double[] SHOOT_ANGLES = {0, 120, 240};
    private static final int[] SHOOT_TO_INTAKE_MAP = {1, 0, 2};
    private static final double MANUAL_INTERVENTION_THRESHOLD = 45.0;
    private static final double SPEED_MULTIPLIER = 0.6;
    private static final double SLOW_SPEED_MULTIPLIER = 0.1;
    private static final double INTAKE_DISTANCE_THRESHOLD = 2.0;
    private static final double SHOOT_DISTANCE_THRESHOLD = 1.5;
    private static final double ANGLE_TOLERANCE = 10.0;

    private final DcMotorEx indexerMotor;
    private ColorRangeSensor intakeColorAndDistanceSensor;
    private ColorRangeSensor shootColorAndDistanceSensor;

    private Servo indexerLight;
    private final ElapsedTime timer;
    private Telemetry telemetry;
    private int intakeIndex = 0;
    private int shootIndex = 0;
    private boolean[] hasBall = {false, false, false};

    public Indexer(HardwareMap hardwareMap, Telemetry telemetry, boolean restMotorPosition) {
        this.indexerMotor = hardwareMap.get(DcMotorEx.class, "indexer");
        this.intakeColorAndDistanceSensor = hardwareMap.get(ColorRangeSensor.class, "fc");
        this.shootColorAndDistanceSensor = hardwareMap.get(ColorRangeSensor.class, "bc");
        this.indexerLight = hardwareMap.get(Servo.class, "light-2");
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

    public void forceFeed(double power) {
        this.indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.indexerMotor.setPower(power * SPEED_MULTIPLIER);
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
     * Explicitly sets the internal state (intakeIndex and shootIndex) to match the current physical motor position.
     * Call this in init() or if you suspect the robot was moved manually while disabled.
     */
    public void syncToPhysicalPosition() {
        double currentAngle = getAngle();
        this.intakeIndex = getClosestIntakeIndex(currentAngle);
        this.shootIndex = getClosestShootIndex(currentAngle);
    }

    private int getClosestIntakeIndex(double currentAngle) {
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

    private int getClosestShootIndex(double currentAngle) {
        int closestIndex = 0;
        double minDifference = Double.MAX_VALUE;

        // Normalize current angle to 0-360
        double normalizedAngle = currentAngle % 360;
        if (normalizedAngle < 0) normalizedAngle += 360;

        for (int i = 0; i < SHOOT_ANGLES.length; i++) {
            double diff = Math.abs(normalizedAngle - SHOOT_ANGLES[i]);
            double diffWrapped = 360 - diff;
            double shortestDiff = Math.min(diff, diffWrapped);

            if (shortestDiff < minDifference) {
                minDifference = shortestDiff;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    public void calculateIntakeIndex() {
        double currentAngle = getAngle();

        // Normalize angle for comparison
        double normalizedAngle = currentAngle % 360;
        if (normalizedAngle < 0) normalizedAngle += 360;

        // Check for manual intervention
        // Calculate distance between current physical angle and what we THINK is the target
        // Calculate distance between current physical angle and what we THINK is the target
        double errorFromState = Math.abs(normalizedAngle - INTAKE_ANGLES[intakeIndex]);
        if (errorFromState > 180) errorFromState = 360 - errorFromState;

        // If error is large, the user moved the motor manually. Sync state first.
        if (errorFromState > MANUAL_INTERVENTION_THRESHOLD) {
            intakeIndex = getClosestIntakeIndex(currentAngle);
        }
    }

    public void calculateShootIndex() {
        double currentAngle = getAngle();

        // Normalize angle for comparison
        double normalizedAngle = currentAngle % 360;
        if (normalizedAngle < 0) normalizedAngle += 360;

        // Check for manual intervention
        double errorFromState = Math.abs(normalizedAngle - SHOOT_ANGLES[shootIndex]);
        if (errorFromState > 180) errorFromState = 360 - errorFromState;

        if (errorFromState > MANUAL_INTERVENTION_THRESHOLD) {
            shootIndex = getClosestShootIndex(currentAngle);
        }
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
        calculateIntakeIndex();

        // Increment index
        intakeIndex = (intakeIndex + 1) % INTAKE_ANGLES.length;

        setTargetAngle(INTAKE_ANGLES[intakeIndex], SPEED_MULTIPLIER);
    }

    public void goToPrevIntakeAngle() {
        calculateIntakeIndex();

        // Decrement index
        intakeIndex = (intakeIndex - 1 + INTAKE_ANGLES.length) % INTAKE_ANGLES.length;

        setTargetAngle(INTAKE_ANGLES[intakeIndex], SPEED_MULTIPLIER);
    }

    public void goToNextEmptyIntakeAngle() {
        calculateIntakeIndex();

        for (int i = 1; i < INTAKE_ANGLES.length; i++) {
            int nextIndex = (intakeIndex + i) % INTAKE_ANGLES.length;
            if (!hasBall[nextIndex]) {
                intakeIndex = nextIndex;
                setTargetAngle(INTAKE_ANGLES[intakeIndex], SPEED_MULTIPLIER);
                updateIntakePos();
                if (!hasBall[intakeIndex]) {
                    this.indexerLight.setPosition(0.0);
                    return;
                }
            }
        }
        this.indexerLight.setPosition(1.0);
    }

    public void goToPrevEmptyIntakeAngle() {
        calculateIntakeIndex();

        for (int i = 1; i < INTAKE_ANGLES.length; i++) {
            int prevIndex = (intakeIndex - i + INTAKE_ANGLES.length) % INTAKE_ANGLES.length;
            if (!hasBall[prevIndex]) {
                intakeIndex = prevIndex;
                setTargetAngle(INTAKE_ANGLES[intakeIndex], SPEED_MULTIPLIER);
                updateIntakePos();
                if (!hasBall[intakeIndex]) {
                    this.indexerLight.setPosition(0.0);
                    return;
                }
            }
        }
        this.indexerLight.setPosition(1.0);
    }

    public void goToNextShootAngle() {
        calculateShootIndex();

        // Increment index
        shootIndex = (shootIndex + 1) % SHOOT_ANGLES.length;

        setTargetAngle(SHOOT_ANGLES[shootIndex], SPEED_MULTIPLIER);
    }

    public void goToPrevShootAngle() {
        calculateShootIndex();

        // Decrement index
        shootIndex = (shootIndex - 1 + SHOOT_ANGLES.length) % SHOOT_ANGLES.length;

        setTargetAngle(SHOOT_ANGLES[shootIndex], SPEED_MULTIPLIER);
    }

    public void goToNextOccupiedShooterAngle() {
        calculateShootIndex();

        for (int i = 1; i < SHOOT_ANGLES.length; i++) {
            int nextShootIndex = (shootIndex + i) % SHOOT_ANGLES.length;
            int associatedIntakeIndex = SHOOT_TO_INTAKE_MAP[nextShootIndex];
            if (hasBall[associatedIntakeIndex]) {
                shootIndex = nextShootIndex;
                setTargetAngle(SHOOT_ANGLES[shootIndex], SPEED_MULTIPLIER);
                updateShootingPos();
                if (hasBall[associatedIntakeIndex]) {
                    return;
                }
            }
        }
    }

    public void goToPrevOccupiedShooterAngle() {
        calculateShootIndex();

        for (int i = 1; i < SHOOT_ANGLES.length; i++) {
            int prevShootIndex = (shootIndex - i + SHOOT_ANGLES.length) % SHOOT_ANGLES.length;
            int associatedIntakeIndex = SHOOT_TO_INTAKE_MAP[prevShootIndex];
            if (hasBall[associatedIntakeIndex]) {
                shootIndex = prevShootIndex;
                setTargetAngle(SHOOT_ANGLES[shootIndex], SPEED_MULTIPLIER);
                updateShootingPos();
                if (hasBall[associatedIntakeIndex]) {
                    return;
                }
            }
        }
    }

    public void resetEncoder() {
        this.indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean isAtAngle(double targetAngle) {
        double currentAngle = getAngle();
        double diff = Math.abs(currentAngle - targetAngle);
        diff = diff % 360;
        if (diff > 180) {
            diff = 360 - diff;
        }
        return diff < ANGLE_TOLERANCE;
    }

    public void updateIntakePos() {
        if (!isAtAngle(INTAKE_ANGLES[intakeIndex])) {
            return;
        }
//        Log.i("Indexer", "Intake position distance: " + intakeColorAndDistanceSensor.getDistance(DistanceUnit.CM));
        if (intakeColorAndDistanceSensor.getDistance(DistanceUnit.CM) < INTAKE_DISTANCE_THRESHOLD) {
            Log.i("Indexer", intakeIndex + " : " + true);
            hasBall[intakeIndex] = true;
        }
    }

    public void updateShootingPos() {
        if (!isAtAngle(SHOOT_ANGLES[shootIndex])) {
            return;
        }
//        Log.i("Indexer", "Shooting position distance: " + shootColorAndDistanceSensor.getDistance(DistanceUnit.CM));
        if (shootColorAndDistanceSensor.getDistance(DistanceUnit.CM) > SHOOT_DISTANCE_THRESHOLD) {
            int associatedIntakeIndex = SHOOT_TO_INTAKE_MAP[shootIndex];
            Log.i("Indexer", shootIndex + " : " + false);
            hasBall[associatedIntakeIndex] = false;
        }
    }

    public boolean[] getBallStatus() {
        return hasBall;
    }

    public double getFrontDistance() {
        return intakeColorAndDistanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getBackDistance() {
        return shootColorAndDistanceSensor.getDistance(DistanceUnit.CM);
    }

    public double geFrontColor() {
        return intakeColorAndDistanceSensor.getLightDetected();
    }

    public double getBackColor() {
        return shootColorAndDistanceSensor.getLightDetected();
    }

    public boolean hasBalls() {
        return hasBall[0] || hasBall[1] || hasBall[2];
    }

    public boolean hasAllBalls() {
        return hasBall[0] && hasBall[1] && hasBall[2];
    }

    public boolean hasBallInShootingPosition() {
        int associatedIntakeIndex = SHOOT_TO_INTAKE_MAP[shootIndex];
        return hasBall[associatedIntakeIndex];
    }

    public boolean hasBallInIntakePosition() {
        return hasBall[intakeIndex];
    }

    public void setAllBallsTrue() {
        hasBall[0] = true;
        hasBall[1] = true;
        hasBall[2] = true;
    }
}
