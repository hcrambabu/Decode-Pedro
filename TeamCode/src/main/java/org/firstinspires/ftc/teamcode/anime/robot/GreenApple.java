package org.firstinspires.ftc.teamcode.anime.robot;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GreenApple {

    private enum RobotState {
        IDLE,
        SPIN_FLY_WHEEL,
        LAUNCH_BALL,
        COLLECT_BALL
    }

    private final Shooter shooter;
    private final Lift lift;
    private final Intake intake;
    private final Indexer indexer;

    private final Limelight limelight;

    private final Timer stateTimer;
    private RobotState robotState = RobotState.IDLE;
    private double flyWheelVelocity = 0;

    public GreenApple(HardwareMap hardwareMap, Telemetry telemetry) {
        shooter = new Shooter(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry, false);
        limelight = new Limelight(hardwareMap, telemetry);
        stateTimer = new Timer();
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Lift getLift() {
        return lift;
    }

    public Intake getIntake() {
        return intake;
    }

    public Indexer getIndexer() {
        return indexer;
    }

    public Limelight getLimelight() {
        return limelight;
    }

    public void setFlyWheelShootingVelocity(double velocity) {
        flyWheelVelocity = velocity;
    }

    public void setRobotState(RobotState state) {
        robotState = state;
        stateTimer.resetTimer();
    }

    public void update() {
        switch (robotState) {
            case IDLE:
                if (indexer.hasBalls() && shooting) {
                    shooter.setVelocity(flyWheelVelocity, true);
                    setRobotState(RobotState.SPIN_FLY_WHEEL);
                }
                break;
            case SPIN_FLY_WHEEL:
                if (shooter.isAtVelocity(flyWheelVelocity)) {
                    indexer.goToNextOccupiedShooterAngle();
                    lift.start(1.0);
                    setRobotState(RobotState.LAUNCH_BALL);
                }
                break;
            case LAUNCH_BALL:
                indexer.updateShootingPos();
                if (!indexer.hasBallInShootingPosition() && stateTimer.getElapsedTime() > 1.0) {
                    if (indexer.hasBalls()) {
                        setRobotState(RobotState.SPIN_FLY_WHEEL);
                    } else {
                        shooter.setVelocity(0, true);
                        lift.stop();
                        intake.start(1.0);
                        indexer.goToNextEmptyIntakeAngle();
                        setRobotState(RobotState.COLLECT_BALL);
                    }
                }
                break;
            case COLLECT_BALL:
                indexer.updateIntakePos();
                if(indexer.hasAllBalls() || stateTimer.getElapsedTime() > 10.0) {
                    intake.stop();
                    setRobotState(RobotState.IDLE);
                } else if (indexer.hasBallInIntakePosition()) {
                    indexer.goToNextEmptyIntakeAngle();
                }
                break;
        }
    }

    private boolean shooting = false;
    public void startShooting() {
        shooting = true;
    }
    public void stopShooting() {
        shooting = false;
    }

    public boolean isShooting() {
        return shooting;
    }

    public boolean isBusy() {
        return robotState != RobotState.IDLE && robotState != RobotState.COLLECT_BALL;
    }
}
