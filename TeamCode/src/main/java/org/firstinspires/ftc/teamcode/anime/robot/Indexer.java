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
    private static final double[] INTAKE_ANGLES = {300, 180, 60};
    private static final double[] SHOOT_ANGLES = {120, 0, 240};
    
    private static final double SPEED_MULTIPLIER = 0.6;
    private static final double SLOW_SPEED_MULTIPLIER = 0.1;
    private static final double INTAKE_LIGHT_THRESHOLD = 0.5; // Tuned based on logs: Intake Empty=0.17, With Ball=1.0
    private static final double SHOOT_LIGHT_THRESHOLD = 0.5; // Tuned based on logs: Shooting Empty=0.2, With Ball=1.0
    private static final double INTAKE_DISTANCE_THRESHOLD = 2.0; // cm
    private static final double SHOOT_DISTANCE_THRESHOLD = 3.0; // cm
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



    private BallColor detectColor(ColorRangeSensor sensor, double lightThreshold, double distanceThreshold) {
        double red = sensor.red();
        double green = sensor.green();
        double blue = sensor.blue();
        double light = sensor.getLightDetected();
        double distance = sensor.getDistance(DistanceUnit.CM);

        // Log.i("Indexer", "detectColor: R=" + red + " G=" + green + " B=" + blue + " Light=" + light + " Dist=" + distance);

        // Dust/Reflection Mitigation:
        // Even if light is low (or high due to dust), if distance is large, it's EMPTY.
        if (distance > distanceThreshold) {
            return BallColor.EMPTY;
        }

        // Standard Light Check (still useful if distance is noisy at close range, but distance is usually primary for presence)
        if (light < lightThreshold) {
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
                if (currentLight > INTAKE_LIGHT_THRESHOLD) { // Presence Threshold
                    // Use Intake Specific Thresholds
                    BallColor c = detectColor(intakeColorAndDistanceSensor, INTAKE_LIGHT_THRESHOLD, INTAKE_DISTANCE_THRESHOLD);
                    if (c != BallColor.EMPTY) {
                        ballColors[i] = c;
                    }
                }
            }
        }
//        Log.i("Indexer", "Balls collected. " + Arrays.toString(ballColors));
        if (this.hasAllBalls()) {
            Log.i("Indexer", "All balls collected. " + Arrays.toString(ballColors));
            this.setLight(1.0);
        } else {
            this.setLight(0.0);
        }
    }

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
        BallColor targetColor = patternQueue.peek(); // CHANGED: Peek instead of poll to preserve queue until shot
        
//        Log.i("Indexer", "find " + targetColor + ", in " + Arrays.toString(ballColors));
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

    public void updateShootingPos() {
        // Prevent state updates while moving (avoids false positives when a ball rotates out of view)
        if (indexerMotor.isBusy()) {
//            Log.i("Indexer", "Indexer busy, skipping shooting position update.");
            return;
        }

        for (int i = 0; i < 3; i++) {
            if (isAtAngle(SHOOT_ANGLES[i])) {
//                Log.i("Indexer", "At shoot angle for slot " + i);
//                Log.i("Indexer", "Shooter Light Detected: " + shootColorAndDistanceSensor.getLightDetected());
                if (shootColorAndDistanceSensor.getLightDetected() < SHOOT_LIGHT_THRESHOLD || shootColorAndDistanceSensor.getDistance(DistanceUnit.CM) > SHOOT_DISTANCE_THRESHOLD) {
                    if (ballColors[i] != BallColor.EMPTY) {
                        // Ball JUST left (Shot confirmed)
                        
                        // Check if this was the pattern ball we were waiting for
                        if (!patternQueue.isEmpty()) {
                            BallColor expected = patternQueue.peek();
                            if (ballColors[i] == expected || expected == BallColor.EMPTY) {
                                patternQueue.poll(); // Consume only on successful shot of correct color
                                Log.i("Indexer", "Shot Pattern Ball: " + expected + ". Queue Size: " + patternQueue.size());
                            }
                        }
                    }
//                    else {
//                        Log.i("Indexer", "No ball to shoot from slot " + i);
//                    }
//                    Log.i("Indexer", "Ball shot from slot " + i);
                    ballColors[i] = BallColor.EMPTY;
                }
//                else {
//                    BallColor detectedColor = detectColor(shootColorAndDistanceSensor, SHOOT_LIGHT_THRESHOLD, SHOOT_DISTANCE_THRESHOLD);
//                    if (detectedColor != BallColor.EMPTY) {
//                        ballColors[i] = detectedColor;
//                        Log.i("Indexer", "Ball detected in slot " + i + ": " + detectedColor);
//                    } else {
//                        Log.i("Indexer", "No ball detected in slot " + i + " despite light reading.");
//                    }
//                }
            }
//            else {
//                Log.i("Indexer", "Not at shoot angle for slot " + i + ", skipping.");
//            }
        }
        if (this.hasAllBalls()) {
            this.setLight(1.0);
        } else {
            this.setLight(0.0);
        }
    }
    
    // Safety Check for TeleOp
    public boolean isReadyToShoot() {
        // 1. Indexer shouldn't be moving
        if (indexerMotor.isBusy()) return false;
        
        int currentSlot = getCurrentShootSlot();
        
        // 2. Must be physically aligned
        if (!isAtAngle(SHOOT_ANGLES[currentSlot])) return false;
        
        // 3. Must have a ball
        if (ballColors[currentSlot] == BallColor.EMPTY) return false;
        
        // 4. If Pattern exists, match it
        if (!patternQueue.isEmpty()) {
            BallColor target = patternQueue.peek();
            // If target is UNKNOWN, any ball is fine. If specific, must match.
            if (target != BallColor.EMPTY && ballColors[currentSlot] != target && hasColor(target)) {
                return false;
            }
        } else {
            // No pattern means we shouldn't shoot
            return false;
        }
        
        return true;
    }

    private boolean hasColor(BallColor target) {
        for (BallColor c : ballColors) {
            if (c == target) {
                return true;
            }
        }
        return false;
    }

    // Telemetry Helper
    public String getPatternStatus() {
        if (patternQueue.isEmpty()) return "Queue: [] (Done)";
        return "Queue: " + patternQueue.toString();
    }

    // --- Getters ---

    public BallColor[] getBallStatus() {
        return ballColors;
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
        ballColors[0] = BallColor.GREEN;
        ballColors[1] = BallColor.PURPLE;
        ballColors[2] = BallColor.PURPLE;
    }

    public void clearAllBalls() {
        ballColors[0] = BallColor.EMPTY;
        ballColors[1] = BallColor.EMPTY;
        ballColors[2] = BallColor.EMPTY;
        this.emptyPatternQueue();
    }

    public ColorRangeSensor getIntakeColorAndDistanceSensor() {
        return intakeColorAndDistanceSensor;
    }
    public ColorRangeSensor getShootColorAndDistanceSensor() {
        return shootColorAndDistanceSensor;
    }
}
