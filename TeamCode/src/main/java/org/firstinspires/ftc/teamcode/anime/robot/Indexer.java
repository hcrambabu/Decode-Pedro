package org.firstinspires.ftc.teamcode.anime.robot;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Objects;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Indexer {
    private static final double TICKS_PER_REV = 751.8;
    private static final double TIME_OUT = 1.0;
    
    // Slot Configuration:
    // Slot 0: Intake at 60, Shoot at 120
    // Slot 1: Intake at 180, Shoot at 0
    // Slot 2: Intake at 300, Shoot at 240
    private static final double[] INTAKE_ANGLES = {60, 180, 300};
    private static final double[] SHOOT_ANGLES = {120, 0, 240};
    
    private static final double SPEED_MULTIPLIER = 0.6;
    private static final double SLOW_SPEED_MULTIPLIER = 0.1;
    private static final double LIGHT_THRESHOLD = 0.4; // Tuned based on logs: Empty=0.17, Ball=1.0
    private static final double ANGLE_TOLERANCE = 10.0;

    private final DcMotorEx indexerMotor;
    private final ColorRangeSensor intakeColorAndDistanceSensor;
    private final ColorRangeSensor shootColorAndDistanceSensor;

    private final Servo indexerLight;
    private final ElapsedTime timer;
    private Telemetry telemetry;

    private final BallColor[] ballColors = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
    private final LinkedList<BallColor> patternQueue = new LinkedList<>();

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

    public double getAngle() {
        return (indexerMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    public void resetEncoder() {
        this.indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // --- Helpers ---

    private double normalizeAngle(double angle) {
        double normalized = angle % 360;
        if (normalized < 0) normalized += 360;
        return normalized;
    }

    private int getClosestSlot(double targetAngle, double[] angleArr) {
        int closestIndex = 0;
        double minDifference = Double.MAX_VALUE;
        double currentAngle = normalizeAngle(targetAngle);

        for (int i = 0; i < angleArr.length; i++) {
            double diff = Math.abs(currentAngle - angleArr[i]);
            double diffWrapped = 360 - diff;
            double shortestDiff = Math.min(diff, diffWrapped);

            if (shortestDiff < minDifference) {
                minDifference = shortestDiff;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    private int getCurrentIntakeSlot() {
        return getClosestSlot(getAngle(), INTAKE_ANGLES);
    }

    private int getCurrentShootSlot() {
        return getClosestSlot(getAngle(), SHOOT_ANGLES);
    }

    private void setTargetAngle(double targetAngleDegrees, double power) {
        double currentAngle = getAngle();
        double difference = targetAngleDegrees - currentAngle;
        double delta = (difference % 360 + 360) % 360;
        if (delta > 180) delta -= 360;

        int currentPosTicks = indexerMotor.getCurrentPosition();
        int deltaTicks = (int) ((delta / 360.0) * TICKS_PER_REV);
        int targetTicks = currentPosTicks + deltaTicks;

//        Log.i("Indexer", "Target: " + targetAngleDegrees + ", Delta: " + delta + ", Ticks: " + targetTicks);
        indexerMotor.setTargetPosition(targetTicks);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexerMotor.setPower(power);
        timer.reset();
    }

    // --- Navigation ---

    public void goToNextIntakeAngle() {
        int currentSlot = getCurrentIntakeSlot();
        int nextSlot = (currentSlot + 1) % 3;
        setTargetAngle(INTAKE_ANGLES[nextSlot], SPEED_MULTIPLIER);
    }

    public void goToPrevIntakeAngle() {
        int currentSlot = getCurrentIntakeSlot();
        int prevSlot = (currentSlot - 1 + 3) % 3;
        setTargetAngle(INTAKE_ANGLES[prevSlot], SPEED_MULTIPLIER);
    }

    public void goToNextEmptyIntakeAngle() {
        int currentSlot = getCurrentIntakeSlot();
        // Check next slots for empty space
        for (int i = 1; i < 3; i++) {
            int checkSlot = (currentSlot + i) % 3;
            if (ballColors[checkSlot] == BallColor.EMPTY) {
                setTargetAngle(INTAKE_ANGLES[checkSlot], SPEED_MULTIPLIER);
                return;
            }
        }
        // If all full, maybe just turn on light or stay put
        this.indexerLight.setPosition(1.0);
    }
    
    public void goToPrevEmptyIntakeAngle() {
        int currentSlot = getCurrentIntakeSlot();
        for (int i = 1; i < 3; i++) {
            int checkSlot = (currentSlot - i + 3) % 3;
            if (ballColors[checkSlot] == BallColor.EMPTY) {
                setTargetAngle(INTAKE_ANGLES[checkSlot], SPEED_MULTIPLIER);
                return;
            }
        }
        this.indexerLight.setPosition(1.0);
    }

    public void goToNextShootAngle() {
        int currentSlot = getCurrentShootSlot();
        int nextSlot = (currentSlot + 1) % 3;
        setTargetAngle(SHOOT_ANGLES[nextSlot], SPEED_MULTIPLIER);
    }
    
    public void goToPrevShootAngle() {
        int currentSlot = getCurrentShootSlot();
        int prevSlot = (currentSlot - 1 + 3) % 3;
        setTargetAngle(SHOOT_ANGLES[prevSlot], SPEED_MULTIPLIER);
    }

    private boolean isAtAngle(double targetAngle) {
        double current = getAngle();
        double diff = Math.abs(current - targetAngle) % 360;
        if (diff > 180) diff = 360 - diff;
        return diff < ANGLE_TOLERANCE;
    }



    private BallColor detectColor(ColorRangeSensor sensor) {
        double red = sensor.red();
        double green = sensor.green();
        double blue = sensor.blue();
        double light = sensor.getLightDetected();

        // Log.i("Indexer", "detectColor: ColorSensor: R=" + red + " G=" + green + " B=" + blue + " Light=" + light);

        if (light < LIGHT_THRESHOLD) { 
            return BallColor.EMPTY;
        }

        if (green > red && green > blue) {
            return BallColor.GREEN;
        } else {
            return BallColor.PURPLE;
        }
    }

    public void updateIntakePos() {
        for (int i = 0; i < 3; i++) {
            if (isAtAngle(INTAKE_ANGLES[i])) {
                double currentLight = intakeColorAndDistanceSensor.getLightDetected();
                if (currentLight > LIGHT_THRESHOLD) { // Presence Threshold
                    BallColor c = detectColor(intakeColorAndDistanceSensor);
                    if (c != BallColor.EMPTY) {
                        ballColors[i] = c;
                    }
                }
            }
        }
        if (this.hasAllBalls()) {
            Log.i("Indexer", "All balls collected. " + Arrays.toString(ballColors));
            this.setLight(1.0);
        } else {
            this.setLight(0.0);
        }
    }

    public void updateShootingPos() {
        // Prevent state updates while moving (avoids false positives when a ball rotates out of view)
        if (indexerMotor.isBusy()) {
            return;
        }

        for (int i = 0; i < 3; i++) {
            if (isAtAngle(SHOOT_ANGLES[i])) {
                if (shootColorAndDistanceSensor.getLightDetected() < LIGHT_THRESHOLD) {
                    ballColors[i] = BallColor.EMPTY;
                }
            }
        }
        if (this.hasAllBalls()) {
            this.setLight(1.0);
        } else {
            this.setLight(0.0);
        }
    }

    // --- Navigation (Pattern Aware) ---
    
    // Restored for compatibility
    public void goToNextOccupiedShooterAngle() {
        int currentSlot = getCurrentShootSlot();
        for (int i = 1; i < 3; i++) {
            int checkSlot = (currentSlot + i) % 3;
            if (ballColors[checkSlot] != BallColor.EMPTY) {
                setTargetAngle(SHOOT_ANGLES[checkSlot], SPEED_MULTIPLIER);
                return;
            }
        }
    }

    public void goToPrevOccupiedShooterAngle() {
        int currentSlot = getCurrentShootSlot();
        for (int i = 1; i < 3; i++) {
            int checkSlot = (currentSlot - i + 3) % 3;
            if (ballColors[checkSlot] != BallColor.EMPTY) {
                setTargetAngle(SHOOT_ANGLES[checkSlot], SPEED_MULTIPLIER);
                return;
            }
        }
    }

    /**
     * Aligns the indexer to the best available ball logic:
     * 1. If we have the ball for the current pattern step -> Go to it.
     * 2. If we don't have the pattern ball -> Go to ANY ball.
     * 3. If we are already at a valid slot -> Stay.
     */
    public void alignToBestBall() {
        if (isBusy(false)) {
            return;
        }
        if (patternQueue.isEmpty()) {
            BallColor[] patt = PoseStorage.patternMap.getOrDefault(org.firstinspires.ftc.teamcode.anime.robot.PoseStorage.pattrenNumber, new BallColor[]{});
            for(BallColor c : patt) {
                patternQueue.offer(c);
            }
        }
        if (patternQueue.isEmpty()) {
            return;
        }
        BallColor targetColor = patternQueue.poll();
        Log.i("Indexer", "find " + targetColor + ", in " + Arrays.toString(ballColors));
        int bestSlot = -1;
        for (int i = 0; i < 3; i++) {
            if (ballColors[i] == targetColor) {
                Log.i("Indexer", "Aligning to best ball of color: " + targetColor + " at slot " + i);
                bestSlot = i;
                break;
            }
        }
        if (bestSlot == -1) {
            // No matching color found, pick any occupied slot
            for (int i = 0; i < 3; i++) {
                if (ballColors[i] != BallColor.EMPTY) {
                    bestSlot = i;
                    break;
                }
            }
        }
        if (bestSlot == -1) {
            // No balls available
            return;
        }
        setTargetAngle(SHOOT_ANGLES[bestSlot], SPEED_MULTIPLIER);
    }

    public void emptyPatternQueue() {
        patternQueue.clear();
    }

    // --- Getters ---

    public boolean[] getBallStatus() {
        boolean[] status = new boolean[3];
        for(int i=0; i<3; i++) status[i] = (ballColors[i] != BallColor.EMPTY);
        return status;
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
    
    // For Telemetry / Debug
    public double getAngleForSlot(int slot, boolean isIntake) {
        if(slot < 0 || slot > 2) return 0;
        return isIntake ? INTAKE_ANGLES[slot] : SHOOT_ANGLES[slot];
    }
    
    public boolean hasAllBalls() {
        return (ballColors[0] != BallColor.EMPTY) && (ballColors[1] != BallColor.EMPTY) && (ballColors[2] != BallColor.EMPTY);
    }

    public boolean hasBalls() {
        return (ballColors[0] != BallColor.EMPTY) || (ballColors[1] != BallColor.EMPTY) || (ballColors[2] != BallColor.EMPTY);
    }
    
    public boolean hasBallInShootingPosition() {
        return ballColors[getCurrentShootSlot()] != BallColor.EMPTY;
    }

    public boolean hasBallInIntakePosition() {
        // Technically "Ball in intake position" means "Is the slot implicitly at the intake populated?"
        // Simpler: Current Intake Slot is Full?
        return ballColors[getCurrentIntakeSlot()] != BallColor.EMPTY;
    }

    public void setLight(double position) {
        this.indexerLight.setPosition(position);
    }

    public void setAllBallsTrue() {
        ballColors[0] = BallColor.PURPLE;
        ballColors[1] = BallColor.PURPLE;
        ballColors[2] = BallColor.PURPLE;
    }
}
