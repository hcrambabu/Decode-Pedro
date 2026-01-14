# Variable Documentation for Decode-Pedro Project

This document explains what each variable does in the TeamCode source files.

---

## Robot Classes

### `Shooter.java`

**Purpose**: Controls the shooter motor for launching game elements.

**Variables**:
- `MAX_VELOCITY` (public static final double = 2000): Maximum velocity in encoder ticks per second that the shooter motor can achieve. Used as a safety limit.
- `shooterMotor` (private final DcMotorEx): The motor controller for the shooter mechanism. Configured to use encoder-based velocity control with PIDF coefficients.
- `telemetry` (private Telemetry): Telemetry object for debugging and displaying shooter status information.

**Methods**:
- `setVelocity(double velocity)`: Sets the target velocity for the shooter motor.
- `getVelocity()`: Returns the current velocity of the shooter motor.
- `getPower()`: Returns the current power being applied to the shooter motor.
- `resetEncoder()`: Resets the encoder position to zero.

---

### `Indexer.java`

**Purpose**: Controls the indexer mechanism that rotates to different positions to hold game elements.

**Constants**:
- `TICKS_PER_REV` (private static final double = 751.8): Number of encoder ticks per full rotation of the indexer motor. Used to convert between encoder ticks and degrees.
- `TIME_OUT` (private static final double = 1.0): Maximum time in seconds to wait for the indexer to reach its target position before timing out.
- `INTAKE_ANGLES` (private static final double[] = {60, 180, 300}): Array of three angles (in degrees) where the indexer can hold game elements. These are the three positions the indexer rotates between.
- `MANUAL_INTERVENTION_THRESHOLD` (private static final double = 45.0): Maximum angle difference (in degrees) between expected and actual position before the system assumes manual intervention occurred. If exceeded, the system resyncs to the physical position.
- `SPEED_MULTIPLIER` (private static final double = 0.6): Multiplier applied to power when moving the indexer at normal speed (60% of input power).
- `SLOW_SPEED_MULTIPLIER` (private static final double = 0.1): Multiplier applied to power when moving the indexer in slow mode (10% of input power).

**Instance Variables**:
- `indexerMotor` (private final DcMotorEx): The motor controller for the indexer mechanism. Set to reverse direction.
- `frontColorSensor` (private ColorRangeSensor): Color/range sensor mounted at the front of the indexer to detect game elements.
- `backColorSensor` (private ColorRangeSensor): Color/range sensor mounted at the back of the indexer to detect game elements.
- `timer` (private final ElapsedTime): Timer used to track how long the indexer has been trying to reach a position (for timeout detection).
- `telemetry` (private Telemetry): Telemetry object for debugging.
- `targetIndex` (private int = 0): Current target position index (0, 1, or 2) corresponding to one of the three `INTAKE_ANGLES`. Tracks which position the indexer should be at.
- `hasBall` (private boolean[] = {false, false, false}): Array tracking whether each of the three indexer positions currently has a game element. Currently declared but not actively used in the visible code.

**Key Methods**:
- `getAngle()`: Calculates current angle in degrees based on encoder position.
- `syncToPhysicalPosition()`: Resyncs the internal state (`targetIndex`) to match the actual physical position of the motor.
- `goToNextIntakeAngle()`: Rotates to the next position in the sequence (0→1→2→0).
- `goToPrevIntakeAngle()`: Rotates to the previous position in the sequence (0→2→1→0).

---

### `Lift.java`

**Purpose**: Controls the lift mechanism for raising/lowering game elements.

**Variables**:
- `liftMotor` (private final DcMotorEx): The motor controller for the lift mechanism. Set to reverse direction.
- `telemetry` (private Telemetry): Telemetry object for debugging.

**Methods**:
- `start(double power)`: Sets the lift motor power using safe power control (slew rate limiting).

---

### `Intake.java`

**Purpose**: Controls the intake mechanism for collecting game elements.

**Variables**:
- `intakeMotor` (private final DcMotorEx): The motor controller for the intake mechanism.
- `telemetry` (private Telemetry): Telemetry object for debugging.

**Methods**:
- `start(double power)`: Sets the intake motor power using safe power control (slew rate limiting).

---

### `Utils.java`

**Purpose**: Utility class providing helper methods for safe motor control.

**Constants**:
- `SLEW_RATE` (public final static double = 0.2): Maximum change in power per update cycle. Used to prevent sudden power changes that could damage motors or cause jerky movement.

**Methods**:
- `setSafePower(DcMotorEx motor, double targetPower)`: Gradually changes motor power to the target value, limiting the rate of change to `SLEW_RATE` per update. This prevents sudden power spikes and provides smoother motor control.

---

## OpModes

### `AnimeTeleop.java`

**Purpose**: Main teleoperated (driver-controlled) mode for the robot.

**Drive Control Variables**:
- `follower` (private Follower): Pedro Pathing follower object that handles mecanum drive control and localization.
- `telemetryM` (private TelemetryManager): Advanced telemetry manager for displaying debug information.
- `driveInSlowMode` (private boolean = false): Flag indicating whether slow mode is currently active for drive control.
- `driveSlowModeMultiplier` (private double = 0.2): Multiplier applied to drive inputs when slow mode is active (20% of normal speed).
- `slowModeIncrement` (private double = 0.2): Amount to increase/decrease `driveSlowModeMultiplier` when adjusting slow mode strength.
- `automatedDrive` (private boolean = false): Flag indicating whether automated path following is active (currently commented out).

**Robot Subsystem Variables**:
- `shooter` (private Shooter): Shooter subsystem instance.
- `lift` (private Lift): Lift subsystem instance.
- `intake` (private Intake): Intake subsystem instance.
- `indexer` (private Indexer): Indexer subsystem instance.
- `indexerSlow` (private boolean = false): Flag indicating whether indexer is in slow mode.

**Shooter Control Variables**:
- `shooterVelocityPreset` (private int = 3): Current selected preset index (0-3) for shooter velocity.
- `shooterVelocityPresets` (private double[] = {1000, 1200, 1500, 1650}): Array of four velocity presets (in encoder ticks per second) that can be selected for the shooter.

**Input State Tracking**:
- `dpadLeftWasPressed` (private boolean = false): Previous frame state of left D-pad button (for edge detection).
- `dpadRightWasPressed` (private boolean = false): Previous frame state of right D-pad button (for edge detection).

**Key Behaviors**:
- Gamepad 1 controls drive (left stick for translation, right stick X for rotation).
- Gamepad 2 controls robot subsystems (shooter, lift, intake, indexer).
- Right bumper on gamepad 1 toggles slow mode.
- X/Y buttons on gamepad 1 adjust slow mode strength.
- Left/Right bumpers on gamepad 2 cycle through shooter velocity presets.
- Right trigger on gamepad 2 activates shooter at selected preset velocity.
- Left trigger on gamepad 2 controls lift.
- Right stick Y on gamepad 2 controls intake.
- Left stick X on gamepad 2 manually controls indexer.
- D-pad left/right on gamepad 2 moves indexer to previous/next position.
- X button on gamepad 2 toggles indexer slow mode.

---

### `ResetEncoders.java`

**Purpose**: Utility opmode to reset all encoder positions to zero.

**Variables**:
- `shooter` (private Shooter): Shooter subsystem instance.
- `lift` (private Lift): Lift subsystem instance.
- `intake` (private Intake): Intake subsystem instance.
- `indexer` (private Indexer): Indexer subsystem instance.

**Behavior**: Resets encoders on shooter, indexer, and all four drive motors (rf, rb, lf, lb).

---

### Auto OpModes (AnimeAutoLeft, AnimeAutoCenter, AnimeAutoRight, AnimeAutoRed, AnimeAutoBlue)

**Purpose**: Autonomous opmodes for different starting positions and alliance colors.

**Variables**: None currently (all are empty templates with placeholder comments).

**Note**: These opmodes are currently stubs and need to be implemented with actual autonomous routines.

---

## Pathing Classes

### `Constants.java`

**Purpose**: Central configuration file for Pedro Pathing follower setup.

**Constants**:
- `followerConstants` (public static FollowerConstants): Configuration for the follower's physical properties.
  - `.mass(12.8)`: Robot mass in pounds (used for physics calculations).
  - Commented: forward and lateral zero power acceleration settings.

- `driveConstants` (public static MecanumConstants): Configuration for the mecanum drivetrain.
  - `.maxPower(1)`: Maximum power allowed for drive motors (100%).
  - `.rightFrontMotorName("rf")`: Hardware map name for right front motor.
  - `.rightRearMotorName("rb")`: Hardware map name for right rear motor.
  - `.leftRearMotorName("lb")`: Hardware map name for left rear motor.
  - `.leftFrontMotorName("lf")`: Hardware map name for left front motor.
  - `.leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)`: Direction setting for left front motor.
  - `.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)`: Direction setting for left rear motor.
  - `.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)`: Direction setting for right front motor.
  - `.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)`: Direction setting for right rear motor.
  - Commented: X and Y velocity settings.

- `localizerConstants` (public static PinpointConstants): Configuration for the odometry/localization system.
  - `.forwardPodY(-5.9375)`: Y position offset of the forward odometry pod in inches.
  - `.strafePodX(-5.1875)`: X position offset of the strafe odometry pod in inches.
  - `.distanceUnit(DistanceUnit.INCH)`: Unit of measurement for distances.
  - `.hardwareMapName("pinpoint")`: Hardware map name for the odometry pods.
  - `.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)`: Encoder resolution constant for GoBilda 4-bar pod.
  - `.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)`: Direction of forward encoder.
  - `.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)`: Direction of strafe encoder.

- `pathConstraints` (public static PathConstraints = new PathConstraints(0.99, 100, 1, 1)): Constraints for path following.
  - First parameter (0.99): Maximum velocity (99% of max).
  - Second parameter (100): Maximum acceleration.
  - Third parameter (1): Maximum angular velocity.
  - Fourth parameter (1): Maximum angular acceleration.

**Methods**:
- `createFollower(HardwareMap hardwareMap)`: Factory method that creates and configures a Follower instance with all the constants above.

---

### `Tuning.java`

**Purpose**: Comprehensive tuning opmode with multiple sub-opmodes for calibrating the path following system.

**Static Variables**:
- `follower` (public static Follower): Shared follower instance used by all tuning opmodes.
- `poseHistory` (static PoseHistory, @IgnoreConfigurable): History of robot poses for visualization.
- `telemetryM` (static TelemetryManager, @IgnoreConfigurable): Telemetry manager for all tuning opmodes.
- `changes` (static ArrayList<String>, @IgnoreConfigurable): List of changes made during tuning sessions.

**Tuning OpMode Classes** (all nested within Tuning.java):

#### `LocalizationTest`
**Purpose**: Tests robot localization by allowing manual drive while displaying pose.

**Variables**: None (uses static follower and telemetryM).

---

#### `ForwardTuner`
**Purpose**: Calibrates forward encoder ticks-to-inches conversion.

**Variables**:
- `DISTANCE` (public static double = 48): Target distance in inches to move forward for calibration.

---

#### `LateralTuner`
**Purpose**: Calibrates lateral (strafe) encoder ticks-to-inches conversion.

**Variables**:
- `DISTANCE` (public static double = 48): Target distance in inches to strafe right for calibration.

---

#### `TurnTuner`
**Purpose**: Calibrates turning encoder ticks-to-radians conversion.

**Variables**:
- `ANGLE` (public static double = 2 * Math.PI): Target angle in radians (one full rotation) for calibration.

---

#### `ForwardVelocityTuner`
**Purpose**: Measures maximum forward velocity of the robot.

**Variables**:
- `velocities` (private final ArrayList<Double>): List storing the most recent velocity measurements.
- `DISTANCE` (public static double = 48): Distance in inches to travel forward at full power.
- `RECORD_NUMBER` (public static double = 10): Number of recent velocity measurements to average.
- `end` (private boolean): Flag indicating whether the robot has finished the test.

---

#### `LateralVelocityTuner`
**Purpose**: Measures maximum lateral (strafe) velocity of the robot.

**Variables**:
- `velocities` (private final ArrayList<Double>): List storing the most recent velocity measurements.
- `DISTANCE` (public static double = 48): Distance in inches to strafe left at full power.
- `RECORD_NUMBER` (public static double = 10): Number of recent velocity measurements to average.
- `end` (private boolean): Flag indicating whether the robot has finished the test.

---

#### `ForwardZeroPowerAccelerationTuner`
**Purpose**: Measures natural deceleration (zero power acceleration) in the forward direction.

**Variables**:
- `accelerations` (private final ArrayList<Double>): List storing deceleration measurements.
- `VELOCITY` (public static double = 30): Target velocity in inches per second to reach before cutting power.
- `previousVelocity` (private double): Previous velocity measurement for calculating acceleration.
- `previousTimeNano` (private long): Previous time measurement in nanoseconds for calculating acceleration.
- `stopping` (private boolean): Flag indicating whether the robot has reached target velocity and is now decelerating.
- `end` (private boolean): Flag indicating whether the robot has finished the test.

---

#### `LateralZeroPowerAccelerationTuner`
**Purpose**: Measures natural deceleration (zero power acceleration) in the lateral direction.

**Variables**:
- `accelerations` (private final ArrayList<Double>): List storing deceleration measurements.
- `VELOCITY` (public static double = 30): Target velocity in inches per second to reach before cutting power.
- `previousVelocity` (private double): Previous velocity measurement for calculating acceleration.
- `previousTimeNano` (private long): Previous time measurement in nanoseconds for calculating acceleration.
- `stopping` (private boolean): Flag indicating whether the robot has reached target velocity and is now decelerating.
- `end` (private boolean): Flag indicating whether the robot has finished the test.

---

#### `TranslationalTuner`
**Purpose**: Tunes translational PIDF controllers by keeping robot in place while allowing lateral pushing.

**Variables**:
- `DISTANCE` (public static double = 40): Distance in inches for the forward/backward path.
- `forward` (private boolean = true): Flag indicating whether robot is currently moving forward or backward.
- `forwards` (private Path): Path object for forward movement.
- `backwards` (private Path): Path object for backward movement.

---

#### `HeadingTuner`
**Purpose**: Tunes heading PIDF controller by keeping robot at constant heading while allowing manual turning.

**Variables**:
- `DISTANCE` (public static double = 40): Distance in inches for the forward/backward path.
- `forward` (private boolean = true): Flag indicating whether robot is currently moving forward or backward.
- `forwards` (private Path): Path object for forward movement.
- `backwards` (private Path): Path object for backward movement.

---

#### `DriveTuner`
**Purpose**: Tunes drive PIDF controller by running robot forward and backward in a straight line.

**Variables**:
- `DISTANCE` (public static double = 40): Distance in inches for the forward/backward path.
- `forward` (private boolean = true): Flag indicating whether robot is currently moving forward or backward.
- `forwards` (private PathChain): PathChain object for forward movement.
- `backwards` (private PathChain): PathChain object for backward movement.

---

#### `Line`
**Purpose**: Tests all PIDF controllers by running robot forward and backward while correcting for disturbances.

**Variables**:
- `DISTANCE` (public static double = 40): Distance in inches for the forward/backward path.
- `forward` (private boolean = true): Flag indicating whether robot is currently moving forward or backward.
- `forwards` (private Path): Path object for forward movement.
- `backwards` (private Path): Path object for backward movement.

---

#### `CentripetalTuner`
**Purpose**: Tests centripetal force correction by running robot in curved paths.

**Variables**:
- `DISTANCE` (public static double = 20): Distance in inches for the curved path.
- `forward` (private boolean = true): Flag indicating whether robot is currently moving forward or backward along the curve.
- `forwards` (private Path): Path object for forward curved movement.
- `backwards` (private Path): Path object for backward curved movement.

---

#### `Triangle`
**Purpose**: Autonomous test that runs robot in a triangular path pattern.

**Variables**:
- `startPose` (private final Pose): Starting position and heading (72, 72, 0 radians).
- `interPose` (private final Pose): Intermediate position and heading (96, 48, π/2 radians).
- `endPose` (private final Pose): End position and heading (96, 96, π/4 radians).
- `triangle` (private PathChain): PathChain containing the three segments of the triangle.

---

#### `Circle`
**Purpose**: Autonomous test that runs robot in a roughly circular path.

**Variables**:
- `RADIUS` (public static double = 10): Radius of the circular path in inches.
- `circle` (private PathChain): PathChain containing the circular path segments.

---

#### `Drawing`
**Purpose**: Utility class for drawing robot position and paths on the Panels dashboard.

**Constants**:
- `ROBOT_RADIUS` (public static final double = 9): Radius in inches used to draw the robot as a circle.

**Static Variables**:
- `panelsField` (private static final FieldManager): Field manager for Panels dashboard drawing.
- `robotLook` (private static final Style): Style for drawing the robot (blue color, 75% opacity).
- `historyLook` (private static final Style): Style for drawing pose history (green color, 75% opacity).

---

### `SampleAutoPathing.java` (in pedroPathing package)

**Purpose**: Empty class (currently just a stub).

**Variables**: None.

---

### `SampleAutoPathing.java` (in root package)

**Purpose**: Example/template for autonomous pathing (currently mostly commented out).

**Commented Variables** (for reference):
- `follower` (private Follower): Follower instance for path following.
- `pathTimer` (private Timer): Timer for tracking path execution time.
- `opModeTimer` (private Timer): Timer for tracking total opmode runtime.
- `pathState` (PathState enum): Current state in the autonomous routine.
- `startpose` (private final Pose): Starting position and heading.
- `shootPose` (private final Pose): Position and heading for shooting.
- `endPose` (private final Pose): Final position and heading.
- `driveStartPosShootPos` (private PathChain): PathChain from start to shoot position.

**PathState Enum Values**:
- `DRIVE_STARTPOS_SHOOT_POS`: State for driving from start to shoot position.
- `SHOOT_PRELOAD`: State for shooting the preloaded game element.

---

## Summary

This codebase implements a FIRST Tech Challenge (FTC) robot controller with:

1. **Robot Subsystems**: Shooter, Indexer, Lift, and Intake mechanisms
2. **Drive System**: Mecanum drive with Pedro Pathing for advanced path following
3. **Localization**: Odometry-based position tracking using GoBilda pinpoint pods
4. **Teleop Control**: Dual gamepad control with slow mode and preset selection
5. **Tuning Tools**: Comprehensive suite of opmodes for calibrating and tuning the path following system

All variables are documented above with their purpose, type, default values, and usage context.

