# DECODE Autonomous Program Design
## 2025-2026 FIRST Tech Challenge

---

## 1. HIGH-LEVEL STRATEGY

### Core Objectives (Priority Order)
1. **CLASSIFY Preload** (20 pts) - Score preload through GOAL
2. **LEAVE** (5 pts) - Fully cross LAUNCH LINE
3. **CLASSIFY Additional** (20 pts) - Score one more ARTIFACT if time allows
4. **Safe Park** - End in optimal TeleOp position

### Strategy Overview
- **Phase 1 (0-8s)**: Score preload, cross LAUNCH LINE
- **Phase 2 (8-20s)**: Collect and score additional ARTIFACT (if time permits)
- **Phase 3 (20-30s)**: Final positioning and safety shutdown

### Key Design Principles
- **Consistency First**: Prioritize reliable 25 points over risky 45 points
- **Time Management**: Abort risky actions if < 10 seconds remain
- **Localization**: Odometry primary, AprilTag correction only near GOAL
- **Vision Fallback**: MOTIF detection with default path if vision fails
- **Safety**: Robot must be motionless at AUTO end

---

## 2. STATE MACHINE DIAGRAM

```
┌─────────────────────────────────────────────────────────────┐
│                    INITIALIZATION                           │
│  - Initialize Follower, Shooter, Indexer, Intake, Lift      │
│  - Detect MOTIF (AprilTag 21-23) or use default            │
│  - Set starting pose based on alliance and position         │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    PRELOAD_SCORE                            │
│  - Rotate indexer to preload position                       │
│  - Drive to optimal shooting position (12-18" from GOAL)    │
│  - Use AprilTag (ID 20/24) for final pose correction        │
│  - Fire shooter at preset velocity                          │
│  - Verify shot (optional: color sensor check)               │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    CROSS_LAUNCH_LINE                        │
│  - Drive backward to fully cross LAUNCH LINE                │
│  - Verify position (odometry check)                         │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
         ┌───────────┴───────────┐
         │                       │
         │  Time Check           │
         │  < 10s remaining?      │
         │                       │
    YES  │                       │  NO
         │                       │
         ▼                       ▼
┌──────────────────┐    ┌─────────────────────────────────────┐
│   SAFE_PARK      │    │   COLLECT_ARTIFACT                  │
│  - Drive to safe │    │  - Drive to artifact pickup zone   │
│    parking spot  │    │  - Activate intake                   │
│  - Stop all      │    │  - Index artifact into indexer      │
│    subsystems    │    │  - Verify artifact collected         │
└──────────────────┘    └──────────────┬──────────────────────┘
                                       │
                                       ▼
                          ┌─────────────────────────────────────┐
                          │   SCORE_ADDITIONAL                  │
                          │  - Drive to shooting position       │
                          │  - Use AprilTag for pose correction │
                          │  - Fire shooter                     │
                          │  - Verify shot                      │
                          └──────────────┬──────────────────────┘
                                         │
                                         ▼
                          ┌─────────────────────────────────────┐
                          │   FINAL_PARK                        │
                          │  - Drive to safe parking position   │
                          │  - Stop all motors                  │
                          │  - Ensure motionless                │
                          └─────────────────────────────────────┘
```

---

## 3. PSEUDOCODE

```java
public class AnimeAutoDecode extends LinearOpMode {
    // Hardware
    Follower follower;
    Shooter shooter;
    Indexer indexer;
    Intake intake;
    Lift lift;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    
    // State Management
    enum AutoState {
        INIT,
        DETECT_MOTIF,
        PRELOAD_SCORE,
        CROSS_LAUNCH_LINE,
        COLLECT_ARTIFACT,
        SCORE_ADDITIONAL,
        FINAL_PARK,
        SAFE_PARK,
        END
    }
    AutoState currentState = AutoState.INIT;
    
    // Timing
    ElapsedTime autoTimer;
    double TIME_LIMIT = 30.0;
    double SAFE_ABORT_TIME = 10.0;
    
    // Alliance Configuration
    boolean isRedAlliance;
    int goalAprilTagId;  // 24 for red, 20 for blue
    Pose startPose;
    Pose goalPose;
    Pose launchLinePose;
    Pose artifactPickupPose;
    Pose parkPose;
    
    // MOTIF Detection
    int detectedMotif = -1;  // 21, 22, or 23, or -1 for unknown
    
    @Override
    public void runOpMode() {
        initialize();
        
        while (opModeIsActive() && currentState != AutoState.END) {
            updateTimer();
            
            if (getRemainingTime() < SAFE_ABORT_TIME && 
                currentState != AutoState.FINAL_PARK && 
                currentState != AutoState.SAFE_PARK) {
                // Emergency abort to safe park
                currentState = AutoState.SAFE_PARK;
            }
            
            switch (currentState) {
                case INIT:
                    handleInit();
                    break;
                case DETECT_MOTIF:
                    handleDetectMotif();
                    break;
                case PRELOAD_SCORE:
                    handlePreloadScore();
                    break;
                case CROSS_LAUNCH_LINE:
                    handleCrossLaunchLine();
                    break;
                case COLLECT_ARTIFACT:
                    handleCollectArtifact();
                    break;
                case SCORE_ADDITIONAL:
                    handleScoreAdditional();
                    break;
                case FINAL_PARK:
                    handleFinalPark();
                    break;
                case SAFE_PARK:
                    handleSafePark();
                    break;
            }
            
            follower.update();
            updateTelemetry();
        }
        
        shutdown();
    }
    
    void initialize() {
        // Initialize Follower
        follower = Constants.createFollower(hardwareMap);
        
        // Initialize Robot Subsystems
        shooter = new Shooter(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry, false);
        intake = new Intake(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        
        // Initialize Vision
        initAprilTag();
        
        // Initialize Timer
        autoTimer = new ElapsedTime();
        
        // Determine Alliance (from gamepad or config)
        isRedAlliance = determineAlliance();  // Implement based on your method
        goalAprilTagId = isRedAlliance ? 24 : 20;
        
        // Set Starting Pose (adjust based on actual field layout)
        // Assuming robot starts at LAUNCH LINE, touching GOAL
        startPose = new Pose(
            isRedAlliance ? 12.0 : 132.0,  // X position (adjust for actual field)
            12.0,                           // Y position (at LAUNCH LINE)
            isRedAlliance ? Math.PI : 0     // Heading toward field center
        );
        
        follower.setStartingPose(startPose);
        
        // Define Key Positions (adjust based on actual field measurements)
        goalPose = new Pose(
            isRedAlliance ? 12.0 : 132.0,   // X position of GOAL
            36.0,                           // Y position (adjust for optimal shooting distance)
            isRedAlliance ? Math.PI : 0      // Heading toward GOAL
        );
        
        launchLinePose = new Pose(
            startPose.getX(),
            6.0,                            // Behind LAUNCH LINE
            startPose.getHeading()
        );
        
        artifactPickupPose = new Pose(
            isRedAlliance ? 60.0 : 84.0,    // Middle of field
            60.0,                           // Y position for artifact pickup
            Math.PI / 2                     // Heading
        );
        
        parkPose = new Pose(
            isRedAlliance ? 24.0 : 120.0,   // Safe parking position
            24.0,                           // Y position
            Math.PI / 2                     // Heading
        );
        
        currentState = AutoState.DETECT_MOTIF;
    }
    
    void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
            .setDecimation(2)  // Balance range vs. frame rate
            .build();
        
        visionPortal = new VisionPortal.Builder()
            .setCamera(BuiltinCameraDirection.BACK)  // or WebcamName
            .addProcessor(aprilTag)
            .build();
    }
    
    void handleDetectMotif() {
        // Detect OBELISK AprilTag (21, 22, or 23)
        List<AprilTagDetection> detections = aprilTag.getDetections();
        
        for (AprilTagDetection detection : detections) {
            int id = detection.id;
            if (id >= 21 && id <= 23) {
                detectedMotif = id;
                telemetry.addData("MOTIF Detected", id);
                break;
            }
        }
        
        // If no detection after 1 second, use default path
        if (autoTimer.seconds() > 1.0 || detectedMotif != -1) {
            currentState = AutoState.PRELOAD_SCORE;
        }
    }
    
    void handlePreloadScore() {
        // Step 1: Prepare indexer for preload
        if (!indexerPrepared) {
            indexer.goToNextIntakeAngle();  // Move to preload position
            indexerPrepared = true;
            return;
        }
        
        // Step 2: Drive to shooting position
        if (!follower.isBusy() && !atShootingPosition) {
            // Create path to shooting position
            PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                    follower.getPose(),
                    goalPose
                ))
                .setLinearHeadingInterpolation(
                    follower.getPose().getHeading(),
                    goalPose.getHeading()
                )
                .build();
            
            follower.followPath(toShoot);
            atShootingPosition = false;
            return;
        }
        
        // Step 3: Fine-tune position using AprilTag
        if (follower.isBusy() == false && !positionCorrected) {
            AprilTagDetection goalTag = findGoalAprilTag();
            
            if (goalTag != null && goalTag.ftcPose.range < 24.0) {
                // Use AprilTag for pose correction
                double correctionX = goalTag.ftcPose.x;
                double correctionY = goalTag.ftcPose.y;
                double correctionYaw = Math.toRadians(goalTag.ftcPose.yaw);
                
                // Apply small correction movement
                Pose correctedPose = new Pose(
                    follower.getPose().getX() + correctionX * 0.1,
                    follower.getPose().getY() + correctionY * 0.1,
                    follower.getPose().getHeading() + correctionYaw * 0.1
                );
                
                // Small adjustment path
                PathChain correction = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), correctedPose))
                    .setConstantHeadingInterpolation(correctedPose.getHeading())
                    .build();
                
                follower.followPath(correction);
            }
            
            positionCorrected = true;
            return;
        }
        
        // Step 4: Fire shooter
        if (follower.isBusy() == false && positionCorrected && !shotFired) {
            // Set shooter to appropriate velocity (adjust based on distance)
            shooter.setVelocity(1500);  // Use calibrated preset
            
            // Wait for shooter to reach velocity
            sleep(500);
            
            // Index preload into shooter
            indexer.goToNextIntakeAngle();  // Release preload
            
            sleep(300);  // Allow time for shot
            
            shooter.setVelocity(0);
            shotFired = true;
            return;
        }
        
        // Step 5: Transition to next state
        if (shotFired) {
            currentState = AutoState.CROSS_LAUNCH_LINE;
            resetStateFlags();
        }
    }
    
    void handleCrossLaunchLine() {
        if (!follower.isBusy()) {
            // Drive backward to cross LAUNCH LINE
            PathChain crossLine = follower.pathBuilder()
                .addPath(new BezierLine(
                    follower.getPose(),
                    launchLinePose
                ))
                .setConstantHeadingInterpolation(follower.getPose().getHeading())
                .build();
            
            follower.followPath(crossLine);
        }
        
        // Verify we've crossed the line
        if (follower.isBusy() == false) {
            double currentY = follower.getPose().getY();
            if (currentY < 12.0) {  // Behind LAUNCH LINE (adjust threshold)
                currentState = AutoState.COLLECT_ARTIFACT;
                resetStateFlags();
            }
        }
    }
    
    void handleCollectArtifact() {
        // Step 1: Drive to artifact pickup zone
        if (!follower.isBusy() && !atPickupZone) {
            PathChain toPickup = follower.pathBuilder()
                .addPath(new BezierLine(
                    follower.getPose(),
                    artifactPickupPose
                ))
                .setLinearHeadingInterpolation(
                    follower.getPose().getHeading(),
                    artifactPickupPose.getHeading()
                )
                .build();
            
            follower.followPath(toPickup);
            atPickupZone = false;
            return;
        }
        
        // Step 2: Activate intake and collect artifact
        if (follower.isBusy() == false && !artifactCollected) {
            intake.start(0.8);  // Run intake
            
            // Use color sensors to detect artifact
            double frontDistance = indexer.getFrontDistance();
            
            if (frontDistance < 5.0) {  // Artifact detected
                // Index artifact into indexer
                indexer.goToNextIntakeAngle();
                sleep(500);
                
                artifactCollected = true;
                intake.start(0);  // Stop intake
            }
            
            // Timeout after 3 seconds
            if (autoTimer.seconds() > 20.0) {
                artifactCollected = true;  // Give up and move on
                intake.start(0);
            }
            
            return;
        }
        
        // Step 3: Transition to scoring
        if (artifactCollected) {
            currentState = AutoState.SCORE_ADDITIONAL;
            resetStateFlags();
        }
    }
    
    void handleScoreAdditional() {
        // Similar to PRELOAD_SCORE but with collected artifact
        // Drive to shooting position
        if (!follower.isBusy() && !atShootingPosition) {
            PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                    follower.getPose(),
                    goalPose
                ))
                .setLinearHeadingInterpolation(
                    follower.getPose().getHeading(),
                    goalPose.getHeading()
                )
                .build();
            
            follower.followPath(toShoot);
            atShootingPosition = false;
            return;
        }
        
        // Fine-tune with AprilTag
        if (follower.isBusy() == false && !positionCorrected) {
            AprilTagDetection goalTag = findGoalAprilTag();
            // Apply correction similar to PRELOAD_SCORE
            positionCorrected = true;
            return;
        }
        
        // Fire shooter
        if (follower.isBusy() == false && positionCorrected && !shotFired) {
            shooter.setVelocity(1500);
            sleep(500);
            indexer.goToNextIntakeAngle();
            sleep(300);
            shooter.setVelocity(0);
            shotFired = true;
            return;
        }
        
        // Transition to park
        if (shotFired) {
            currentState = AutoState.FINAL_PARK;
            resetStateFlags();
        }
    }
    
    void handleFinalPark() {
        if (!follower.isBusy()) {
            PathChain toPark = follower.pathBuilder()
                .addPath(new BezierLine(
                    follower.getPose(),
                    parkPose
                ))
                .setLinearHeadingInterpolation(
                    follower.getPose().getHeading(),
                    parkPose.getHeading()
                )
                .build();
            
            follower.followPath(toPark);
        }
        
        if (follower.isBusy() == false) {
            // Ensure all motors are stopped
            follower.startTeleopDrive();
            follower.setTeleOpDrive(0, 0, 0, true);
            shooter.setVelocity(0);
            intake.start(0);
            lift.start(0);
            
            currentState = AutoState.END;
        }
    }
    
    void handleSafePark() {
        // Emergency park - drive to nearest safe position
        if (!follower.isBusy()) {
            Pose currentPose = follower.getPose();
            Pose safePose = new Pose(
                currentPose.getX(),
                24.0,  // Safe Y position
                currentPose.getHeading()
            );
            
            PathChain toSafe = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, safePose))
                .setConstantHeadingInterpolation(currentPose.getHeading())
                .build();
            
            follower.followPath(toSafe);
        }
        
        if (follower.isBusy() == false) {
            // Stop everything
            follower.startTeleopDrive();
            follower.setTeleOpDrive(0, 0, 0, true);
            shooter.setVelocity(0);
            intake.start(0);
            lift.start(0);
            
            currentState = AutoState.END;
        }
    }
    
    AprilTagDetection findGoalAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == goalAprilTagId) {
                return detection;
            }
        }
        return null;
    }
    
    double getRemainingTime() {
        return TIME_LIMIT - autoTimer.seconds();
    }
    
    void resetStateFlags() {
        atShootingPosition = false;
        positionCorrected = false;
        shotFired = false;
        atPickupZone = false;
        artifactCollected = false;
        indexerPrepared = false;
    }
    
    void shutdown() {
        visionPortal.close();
        // All subsystems already stopped in FINAL_PARK or SAFE_PARK
    }
    
    // State flags
    boolean atShootingPosition = false;
    boolean positionCorrected = false;
    boolean shotFired = false;
    boolean atPickupZone = false;
    boolean artifactCollected = false;
    boolean indexerPrepared = false;
}
```

---

## 4. TRAJECTORY DEFINITIONS (RoadRunner-Style)

### Field Coordinate System
- Origin (0, 0) at bottom-left corner (from driver perspective)
- X-axis: Left to Right (0 to 144 inches)
- Y-axis: Bottom to Top (0 to 144 inches)
- Heading: 0 = Right, π/2 = Up, π = Left, -π/2 = Down

### Key Field Positions (Red Alliance Example)

```java
// Starting Positions (adjust based on actual field layout)
Pose RED_START_LEFT = new Pose(12.0, 12.0, Math.PI);      // Left start
Pose RED_START_CENTER = new Pose(12.0, 36.0, Math.PI);    // Center start
Pose RED_START_RIGHT = new Pose(12.0, 60.0, Math.PI);    // Right start

Pose BLUE_START_LEFT = new Pose(132.0, 12.0, 0);         // Left start
Pose BLUE_START_CENTER = new Pose(132.0, 36.0, 0);       // Center start
Pose BLUE_START_RIGHT = new Pose(132.0, 60.0, 0);        // Right start

// GOAL Positions (adjust based on actual field measurements)
Pose RED_GOAL = new Pose(12.0, 36.0, Math.PI);           // Red GOAL center
Pose BLUE_GOAL = new Pose(132.0, 36.0, 0);               // Blue GOAL center

// Shooting Positions (12-18 inches from GOAL)
Pose RED_SHOOT_POSE = new Pose(24.0, 36.0, Math.PI);     // Optimal shooting distance
Pose BLUE_SHOOT_POSE = new Pose(120.0, 36.0, 0);

// LAUNCH LINE Crossing
Pose RED_LAUNCH_CROSS = new Pose(12.0, 6.0, Math.PI);     // Behind LAUNCH LINE
Pose BLUE_LAUNCH_CROSS = new Pose(132.0, 6.0, 0);

// Artifact Pickup Zones (adjust based on MOTIF and field layout)
Pose ARTIFACT_ZONE_1 = new Pose(60.0, 60.0, Math.PI/2);   // Middle-left
Pose ARTIFACT_ZONE_2 = new Pose(72.0, 60.0, Math.PI/2);  // Center
Pose ARTIFACT_ZONE_3 = new Pose(84.0, 60.0, Math.PI/2); // Middle-right

// Parking Positions
Pose RED_PARK = new Pose(24.0, 24.0, Math.PI/2);         // Safe parking
Pose BLUE_PARK = new Pose(120.0, 24.0, Math.PI/2);
```

### Trajectory Examples

```java
// Trajectory 1: Start → Shooting Position
PathChain startToShoot = follower.pathBuilder()
    .addPath(new BezierLine(
        RED_START_CENTER,      // Start pose
        RED_SHOOT_POSE         // Shooting pose
    ))
    .setLinearHeadingInterpolation(
        RED_START_CENTER.getHeading(),
        RED_SHOOT_POSE.getHeading()
    )
    .build();

// Trajectory 2: Shooting → Launch Line Cross
PathChain shootToLaunch = follower.pathBuilder()
    .addPath(new BezierLine(
        RED_SHOOT_POSE,
        RED_LAUNCH_CROSS
    ))
    .setConstantHeadingInterpolation(RED_SHOOT_POSE.getHeading())
    .build();

// Trajectory 3: Launch Line → Artifact Pickup
PathChain launchToPickup = follower.pathBuilder()
    .addPath(new BezierCurve(
        RED_LAUNCH_CROSS,
        new Pose(36.0, 30.0, Math.PI/2),  // Control point 1
        new Pose(60.0, 50.0, Math.PI/2),   // Control point 2
        ARTIFACT_ZONE_2
    ))
    .setLinearHeadingInterpolation(
        RED_LAUNCH_CROSS.getHeading(),
        ARTIFACT_ZONE_2.getHeading()
    )
    .build();

// Trajectory 4: Artifact Pickup → Shooting (for second shot)
PathChain pickupToShoot = follower.pathBuilder()
    .addPath(new BezierCurve(
        ARTIFACT_ZONE_2,
        new Pose(48.0, 48.0, Math.PI),     // Control point
        RED_SHOOT_POSE
    ))
    .setLinearHeadingInterpolation(
        ARTIFACT_ZONE_2.getHeading(),
        RED_SHOOT_POSE.getHeading()
    )
    .build();

// Trajectory 5: Final Park
PathChain toPark = follower.pathBuilder()
    .addPath(new BezierLine(
        RED_SHOOT_POSE,  // Current position after second shot
        RED_PARK
    ))
    .setLinearHeadingInterpolation(
        RED_SHOOT_POSE.getHeading(),
        RED_PARK.getHeading()
    )
    .build();
```

---

## 5. HARDWARE ASSUMPTIONS

### Drivetrain
- **Type**: Mecanum drive (4 motors: rf, rb, lf, lb)
- **Localization**: GoBilda Pinpoint 4-bar odometry pods
  - Forward pod Y offset: -5.9375 inches
  - Strafe pod X offset: -5.1875 inches
- **IMU**: Built-in or external IMU for heading
- **Max Power**: 1.0 (100%)
- **Mass**: 12.8 lbs (as configured in Constants.java)

### Scoring System
- **Shooter**: 
  - DcMotorEx with encoder
  - Velocity control with PIDF (P=300, I=0, D=0, F=10)
  - Max velocity: 2000 ticks/sec
  - Preset velocities: 1000, 1200, 1500, 1650 ticks/sec
- **Indexer**:
  - DcMotorEx with encoder
  - 3-position system (60°, 180°, 300°)
  - 751.8 ticks per revolution
  - Color/range sensors (front and back) for artifact detection

### Intake System
- **Intake**: DcMotorEx for collecting artifacts
- **Lift**: DcMotorEx for raising/lowering (if needed)

### Vision System
- **Camera**: Built-in phone camera or webcam
- **AprilTag Detection**: 
  - Decimation: 2 (balance range vs. frame rate)
  - GOAL tags: ID 20 (blue), ID 24 (red)
  - OBELISK tags: ID 21, 22, 23 (MOTIF detection)
- **Usage**: 
  - MOTIF detection during INIT (optional)
  - Pose correction near GOAL (within 24 inches)

### Safety Systems
- **Slew Rate Limiting**: 0.2 max power change per cycle (Utils.java)
- **Timeout Protection**: 1.0 second timeout for indexer movements
- **Emergency Abort**: Safe park if < 10 seconds remaining

---

## 6. TIMING BREAKDOWN

### Optimistic Timeline (45 points)
- **0-2s**: Initialize, detect MOTIF
- **2-6s**: Score preload (drive + shoot)
- **6-8s**: Cross LAUNCH LINE
- **8-14s**: Collect additional artifact
- **14-18s**: Score additional artifact
- **18-22s**: Final park
- **22-30s**: Buffer time

### Conservative Timeline (25 points)
- **0-2s**: Initialize, detect MOTIF
- **2-7s**: Score preload (drive + shoot)
- **7-10s**: Cross LAUNCH LINE
- **10-30s**: Safe park and wait

### Time Checkpoints
- **10s remaining**: Abort risky actions, go to SAFE_PARK
- **5s remaining**: Emergency stop, ensure motionless
- **0s**: AUTO period ends

---

## 7. ERROR HANDLING & FALLBACKS

### Vision Failures
- **MOTIF Detection Fails**: Use default artifact pickup zone (center)
- **AprilTag Not Detected**: Use odometry-only positioning (less accurate but functional)
- **Camera Issues**: Skip vision corrections, rely on odometry

### Mechanical Failures
- **Shooter Not Reaching Velocity**: Timeout after 2 seconds, proceed anyway
- **Artifact Not Collected**: Timeout after 3 seconds, skip second score
- **Indexer Stuck**: Manual intervention detection (45° threshold), resync position

### Localization Failures
- **Odometry Drift**: AprilTag correction when available
- **Complete Localization Loss**: Drive to known field position (GOAL or wall), reset pose

### Time Management
- **< 10s Remaining**: Abort collection/scoring, go to SAFE_PARK
- **< 5s Remaining**: Emergency stop all systems
- **State Timeout**: Each state has maximum time limit, auto-advance if exceeded

---

## 8. TESTING CHECKLIST

### Pre-Competition Testing
- [ ] Verify starting pose accuracy (±1 inch tolerance)
- [ ] Calibrate shooter velocity for 12-18" distance
- [ ] Test AprilTag detection range and accuracy
- [ ] Verify LAUNCH LINE crossing detection
- [ ] Test artifact collection reliability
- [ ] Measure actual trajectory execution times
- [ ] Test emergency abort scenarios
- [ ] Verify robot is motionless at AUTO end

### Field-Specific Calibration
- [ ] Measure actual GOAL position
- [ ] Measure actual LAUNCH LINE position
- [ ] Calibrate shooting distance for field lighting
- [ ] Test artifact pickup zones
- [ ] Verify parking positions don't interfere with TeleOp

### Consistency Testing
- [ ] Run autonomous 10+ times, measure success rate
- [ ] Test with different battery levels
- [ ] Test with different field lighting conditions
- [ ] Verify MOTIF detection reliability

---

## 9. IMPLEMENTATION NOTES

### Integration with Existing Codebase
- Uses `Constants.createFollower()` for drivetrain setup
- Uses existing `Shooter`, `Indexer`, `Intake`, `Lift` classes
- Follows Pedro Pathing trajectory structure
- Compatible with existing telemetry system

### Code Organization
- Create separate opmodes for each starting position (Left, Center, Right)
- Create separate opmodes for each alliance (Red, Blue)
- Use `@Configurable` annotations for easy tuning
- Implement state machine as enum with switch statement

### Tuning Parameters
- Shooting velocity: Calibrate based on distance
- Shooting distance: 12-18 inches from GOAL
- Artifact pickup timeout: 3 seconds
- AprilTag correction threshold: 24 inches
- Emergency abort time: 10 seconds remaining

---

## 10. COMPETITION DAY CHECKLIST

### Before Each Match
- [ ] Verify alliance color (red/blue)
- [ ] Verify starting position (left/center/right)
- [ ] Check battery voltage (> 12.5V recommended)
- [ ] Verify camera is working
- [ ] Test shooter velocity
- [ ] Verify indexer positions
- [ ] Check odometry pods are clean and functioning

### During Autonomous
- [ ] Monitor telemetry for state transitions
- [ ] Watch for error messages
- [ ] Verify robot crosses LAUNCH LINE
- [ ] Confirm preload scores successfully

### After Autonomous
- [ ] Review telemetry logs
- [ ] Note any issues for next match
- [ ] Adjust parameters if needed

---

## END OF DOCUMENT

This autonomous design prioritizes **consistency and reliability** over maximum points. The 25-point baseline (CLASSIFY preload + LEAVE) should be achievable in 95%+ of matches, with the additional 20 points (second CLASSIFY) as a bonus when conditions are favorable.

**Remember**: A reliable 25 points is better than an inconsistent 45 points!

