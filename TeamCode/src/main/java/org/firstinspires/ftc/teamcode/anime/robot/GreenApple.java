package org.firstinspires.ftc.teamcode.anime.robot;

import android.util.Log;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class GreenApple {

    private enum RobotState {
        IDLE,
        SPIN_FLY_WHEEL,
        ALIGN_INDEXER, // New state to wait for alignment
        LAUNCH_BALL,
        COLLECT_BALL
    }



    private final Shooter shooter;
    private final Lift lift;
    private final Intake intake;
    private final Indexer indexer;

    private final Limelight limelight;

    private Telemetry telemetry;

    private final Timer stateTimer;
    private RobotState robotState = RobotState.IDLE;
    private double flyWheelVelocity = 0;

    // Track if intake is manually running
    private boolean intakeRunning = false;

    public GreenApple(HardwareMap hardwareMap, Telemetry telemetry, int limelightPipeline, boolean resetIndexers) {
        this.telemetry = telemetry;
        shooter = new Shooter(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry, resetIndexers);
        limelight = new Limelight(hardwareMap, limelightPipeline, telemetry);
        stateTimer = new Timer();
    }

    public Shooter getShooter() { return shooter; }
    public Lift getLift() { return lift; }
    public Intake getIntake() { return intake; }
    public Indexer getIndexer() { return indexer; }
    public Limelight getLimelight() { return limelight; }

    public void setFlyWheelShootingVelocity(double velocity) {
        flyWheelVelocity = velocity;
    }

    public void startShooter() {
        shooter.setVelocity(flyWheelVelocity, true);
    }

    public void setRobotState(RobotState state) {
        robotState = state;
        stateTimer.resetTimer();
    }

    public void updateTelemetry() {
        telemetry.addData("Robot State", robotState.name());
        telemetry.addData("Robot State Timer: ", stateTimer.getElapsedTimeSeconds());
        telemetry.addData("Balls", Arrays.toString(indexer.getBallStatus()));
        telemetry.addData("Intake Running?", intakeRunning);
    }

    public void update() {
        switch (robotState) {
            case IDLE:
//                Log.i("GreenApple", "In IDLE state");
                if (indexer.hasBalls() && shooting) {
                    shooter.setVelocity(flyWheelVelocity, true);
                    indexer.alignToBestBall();
                    setRobotState(RobotState.SPIN_FLY_WHEEL);
                }
                break;
            case SPIN_FLY_WHEEL:
//                Log.i("GreenApple", "In SPIN_FLY_WHEEL state");
                if (shooter.isAtVelocity(flyWheelVelocity)) {
                    indexer.alignToBestBall(); // Use Pattern-Aware Alignment
                    setRobotState(RobotState.ALIGN_INDEXER);
                }
                break;
            case ALIGN_INDEXER:
//                Log.i("GreenApple", "In ALIGN_INDEXER state");
                // Wait until aligned and ready
                if (indexer.isReadyToShoot()) {
                    lift.start(1.0);
                    setRobotState(RobotState.LAUNCH_BALL);
                } else {
                    // Retry alignment command if needed? 
                    // Usually not needed as indexer logic holds target.
                    // But if we timed out?
                    if (stateTimer.getElapsedTimeSeconds() > 2.0) {
                        // Stuck? Skip this ball? Or force next?
                        // For now, infinite wait is safer than shooting wrong.
                    }
                }
                break;
            case LAUNCH_BALL:
//                Log.i("GreenApple", "In LAUNCH_BALL state");
                indexer.updateShootingPos(); // Checks sensor, updates state, consumes pattern queue
                
                // If the ball is gone (detected 'Empty' in updateShootingPos), we are done.
                // But updateShootingPos also sets light.
                // We need to know if the SHOT happened.
                // Simple logic: If current slot is now Empty, we are done.
                if (!indexer.hasBallInShootingPosition()) {
                     // Shot complete!
                     // Wait a friction of a second? 
                     if (stateTimer.getElapsedTimeSeconds() > 0.2) { // Minimum 0.2s to ensure clear exit
                        lift.stop();
                        if (indexer.hasBalls()) {
                            // More balls? Loop back.
                            setRobotState(RobotState.SPIN_FLY_WHEEL);
                        } else {
                            // Empty? Go to intake.
                            //shooter.setVelocity(0, true); // no stop shooter here to allow spin-down, again sping taking too long
                            lift.stop();
                            intake.start(-1.0); // Corrected: Negative for Intake
                            intakeRunning = true;
                            indexer.goToNextEmptyIntakeAngle();
                            setRobotState(RobotState.COLLECT_BALL);
                        }
                     }
                } else if (stateTimer.getElapsedTimeSeconds() > 10.0) {
                    // Timeout (Jam?).
                    indexer.clearAllBalls();
                    lift.stop();
                    // Go to idle or try again?
                    setRobotState(RobotState.IDLE);
                }
                break;
            case COLLECT_BALL:
//                Log.i("GreenApple", "In COLLECT_BALL state");
                indexer.emptyPatternQueue(); // Reset pattern expectation for new batch
                indexer.updateIntakePos();
                if(indexer.hasAllBalls() || stateTimer.getElapsedTimeSeconds() > 10.0) {
                    intake.stop();
                    intakeRunning = false;
                    setRobotState(RobotState.IDLE);
                } else if (indexer.hasBallInIntakePosition()) {
                    indexer.goToNextEmptyIntakeAngle();
                }
                break;
        }

        updateTelemetry();
    }

    private boolean shooting = false;
    public void startShooting() { 
        shooting = true; 
        if (robotState == RobotState.COLLECT_BALL) {
            // Force exit from collection if we need to shoot
            intake.stop();
            intakeRunning = false;
            setRobotState(RobotState.IDLE);
        }
    }
    public void stopShooting() { shooting = false; }
    public boolean isShooting() { return shooting; }
    public boolean isBusy() { return robotState != RobotState.IDLE && robotState != RobotState.COLLECT_BALL; }

    // =========================
    // Intake Control Methods
    // =========================
    public void startIntake() {
        intake.start(-1.0);
        intakeRunning = true;
    }

    public void stopIntake() {
        intake.stop();
        intakeRunning = false;
    }

    public boolean isIntaking() {
        return intakeRunning;
    }
}
