# Architecture Diagram - Commit 3 Implementation

```
┌─────────────────────────────────────────────────────────────────┐
│                         UTIL PACKAGE                            │
│                 (Foundation - No Dependencies)                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐   ┌───────────────────┐   ┌──────────────┐  │
│  │   Alliance   │   │ StartingPosition  │   │ FieldMirror  │  │
│  │    (enum)    │   │      (enum)       │   │   (utility)  │  │
│  ├──────────────┤   ├───────────────────┤   ├──────────────┤  │
│  │ • RED        │   │ • NEAR            │   │ • mirrorPose │  │
│  │ • BLUE       │   │ • FAR             │   │ • mirrorX/Y  │  │
│  │              │   │                   │   │ • blendPoses │  │
│  │ • getColor() │   │ • getBluePose()   │   │ • distance   │  │
│  └──────────────┘   └───────────────────┘   └──────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
                              ▲
                              │ uses
                              │
┌─────────────────────────────────────────────────────────────────┐
│                    OPMODE PACKAGE - BASE                        │
│                    (Abstract Foundation)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│           ┌────────────────────────────────────┐                │
│           │   BaseCompetitionOpMode            │                │
│           │         (abstract)                 │                │
│           ├────────────────────────────────────┤                │
│           │ • Progressive init_loop selection  │                │
│           │   - Stage 1: Select Robot          │                │
│           │   - Stage 2: Select Alliance       │                │
│           │   - Stage 3: Select Position       │                │
│           │   - Stage 4: Ready                 │                │
│           │                                    │                │
│           │ • Common subsystems:               │                │
│           │   - shooter, ballFeed, vision, led │                │
│           │   - follower, imu, telemetryM      │                │
│           │                                    │                │
│           │ • Utilities:                       │                │
│           │   - getStartingPose()              │                │
│           │   - performRelocalization()        │                │
│           │                                    │                │
│           │ • Child class hooks:               │                │
│           │   - onInitialize()                 │                │
│           │   - onStart()                      │                │
│           │   - onStop()                       │                │
│           └────────────────────────────────────┘                │
│                          ▲                                      │
│                          │ extends                              │
└──────────────────────────┼──────────────────────────────────────┘
                           │
                 ┌─────────┴─────────┐
                 │                   │
┌────────────────▼─────┐   ┌─────────▼──────────────┐
│ TeleopDriveSubsystems│   │   Auto1BallPark        │
│      (TeleOp)        │   │   (Autonomous)         │
├──────────────────────┤   ├────────────────────────┤
│ • Driver control     │   │ • State machine:       │
│ • Auto-aim modes     │   │   - MOVE_TO_SHOOT      │
│ • Tracking toggle    │   │   - AUTO_AIM           │
│ • LED control        │   │   - SHOOT              │
│ • Periodic reloc     │   │   - MOVE_TO_PARK       │
│                      │   │   - PARK               │
│ loop() logic:        │   │                        │
│ • handleDriving()    │   │ • Uses Shooter.autoAim │
│ • handleShooter()    │   │ • Fallback mode        │
│ • handleBallFeed()   │   │ • Path generation      │
│ • handleTracking()   │   │ • Periodic reloc       │
└──────────────────────┘   └────────────────────────┘
```

---

## Inheritance & Composition Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    INHERITANCE TREE                         │
└─────────────────────────────────────────────────────────────┘

                        OpMode (FTC SDK)
                            │
                            │ extends
                            ▼
                BaseCompetitionOpMode
                   (our abstract base)
                            │
               ┌────────────┴────────────┐
               │                         │
               │ extends                 │ extends
               ▼                         ▼
    TeleopDriveSubsystems        Auto1BallPark
         (TeleOp)                 (Autonomous)


┌─────────────────────────────────────────────────────────────┐
│                  COMPOSITION DIAGRAM                        │
│             (What's Inside Each OpMode)                     │
└─────────────────────────────────────────────────────────────┘

BaseCompetitionOpMode HAS:
    ├─ MainCharacter character        (enum)
    ├─ Alliance alliance               (enum)
    ├─ StartingPosition position       (enum)
    ├─ Shooter shooter                 (subsystem)
    ├─ BallFeed ballFeed              (subsystem)
    ├─ Vision vision                  (subsystem)
    ├─ LED led                        (subsystem, nullable)
    ├─ Follower follower              (Pedro Pathing)
    ├─ IMU imu                        (hardware)
    └─ TelemetryManager telemetryM    (Panels)

TeleopDriveSubsystems ADDS:
    ├─ Tracking state (boolean)
    ├─ Timer objects (ElapsedTime)
    └─ Button state tracking

Auto1BallPark ADDS:
    ├─ AutoState currentState          (enum)
    ├─ Path objects (Path, PathChain)
    └─ Fallback mode flag
```

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                   INIT_LOOP FLOW                            │
└─────────────────────────────────────────────────────────────┘

Driver Station
      │
      │ DPAD UP/DOWN + A
      ▼
┌──────────────────┐
│ Stage 1: Robot   │ ──► MainCharacter enum selected
└──────────────────┘     │
      │                  ├─ Initializes subsystems
      │ A pressed        │  (Shooter, BallFeed, Vision, LED)
      ▼                  │
┌──────────────────┐     │
│ Stage 2: Alliance│ ──► Alliance enum selected
└──────────────────┘     │
      │                  │
      │ A pressed        │
      ▼                  │
┌──────────────────┐     │
│ Stage 3: Position│ ──► StartingPosition enum selected
└──────────────────┘     │
      │                  │
      │ A pressed        │
      ▼                  ▼
┌──────────────────┐   ┌─────────────────────┐
│ Stage 4: Ready   │   │ Calculate Start Pose│
└──────────────────┘   │ (using FieldMirror) │
      │                └─────────────────────┘
      │ START pressed
      ▼
   BEGIN MATCH


┌─────────────────────────────────────────────────────────────┐
│                 RELOCALIZATION FLOW                         │
└─────────────────────────────────────────────────────────────┘

Every loop() call:
      │
      ▼
┌──────────────────────┐
│ performRelocalization│
└──────────────────────┘
      │
      ├─► Vision.findGoalTag()
      │        │
      │        ├─ No tag? ──► Return (no correction)
      │        │
      │        ├─ Invalid pose? ──► Return (no correction)
      │        │
      │        └─ Valid tag detected
      │                  │
      │                  ▼
      │        calculatePoseFromTag()  (TODO: Implement)
      │                  │
      │                  ▼
      │        FieldMirror.shouldRelocalize()
      │                  │
      │                  ├─ Error too small? ──► Return
      │                  ├─ Error too large? ──► Return
      │                  │
      │                  └─ Within threshold
      │                           │
      │                           ▼
      │                 FieldMirror.blendPoses()
      │                  (70% odom, 30% vision)
      │                           │
      │                           ▼
      └────────────► follower.setPose(corrected)
```

---

## File Dependencies (Import Graph)

```
Alliance.java
└─ (no dependencies)

StartingPosition.java
└─ com.pedropathing.geometry.Pose

FieldMirror.java
├─ com.pedropathing.geometry.Pose
└─ com.bylazar.configurables.annotations.Configurable

BaseCompetitionOpMode.java
├─ Alliance.java
├─ StartingPosition.java
├─ FieldMirror.java
├─ MainCharacter.java (subsystems package)
├─ Shooter.java
├─ BallFeed.java
├─ Vision.java
├─ LED.java
└─ Pedro Pathing (Follower, Pose)

TeleopDriveSubsystems.java
├─ BaseCompetitionOpMode.java (extends)
├─ Shooter.java (for AutoAimConstants)
├─ Vision.java (for AutoAimResult)
└─ All dependencies inherited from base

Auto1BallPark.java
├─ BaseCompetitionOpMode.java (extends)
├─ Vision.java (for AutoAimResult)
├─ FieldMirror.java (for relocalization)
└─ Pedro Pathing (Path, PathChain, BezierCurve)

Shooter.java
├─ CharacterStats.java
└─ com.bylazar.configurables.annotations.Configurable
```

---

## Constants Organization (Panels Dashboard)

```
┌─────────────────────────────────────────────────────────┐
│                PANELS DASHBOARD VIEW                    │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  FieldMirror.RelocalizationConstants                   │
│    ├─ ODOMETRY_TRUST_RATIO = 0.7                       │
│    ├─ MIN_ERROR_THRESHOLD = 3.0                        │
│    └─ MAX_ERROR_THRESHOLD = 24.0                       │
│                                                         │
│  Shooter.Constants                                      │
│    ├─ MIN_POWER_RPM = 1500                             │
│    ├─ MAX_POWER_RPM = 5500                             │
│    ├─ LINEAR_CORRECTION_FACTOR = 18.0                  │
│    └─ ...                                              │
│                                                         │
│  Shooter.AutoAimConstants                              │
│    ├─ SINGLE_SHOT_DURATION = 0.5                       │
│    ├─ AUTO_AIM_SPIN_DOWN_TIME = 3.0                    │
│    ├─ HEADING_P_GAIN = 0.015                           │
│    ├─ HEADING_DEADBAND_DEG = 2.0                       │
│    ├─ MAX_TRACKING_ROTATION = 0.4                      │
│    └─ OVERRIDE_THRESHOLD = 0.1                         │
│                                                         │
│  Auto1BallPark.AutoConstants                           │
│    ├─ SHOOT_X = 72.0                                   │
│    ├─ SHOOT_Y_NEAR = 24.0                              │
│    ├─ SHOOT_Y_FAR = 120.0                              │
│    └─ ...                                              │
│                                                         │
│  Robot22154Abilities.ShooterConstants                  │
│    ├─ HIGH_VELOCITY_RPM = 4000                         │
│    ├─ LOW_VELOCITY_RPM = 1500                          │
│    └─ ...                                              │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## State Machines

### TeleOp Tracking State Machine
```
              ┌─────────────┐
              │   MANUAL    │ ◄──── Default state
              │  (No Track) │
              └─────────────┘
                     │
         Y button    │    LBumper toggle
         pressed     │    pressed
                     │
              ┌──────▼──────────┐
              │  SINGLE-SHOT    │
   Timeout    │   TRACKING      │
   (0.5s) ──► │                 │
              └──────┬──────────┘
                     │
         LBumper     │
         toggle      │
                     │
              ┌──────▼──────────┐
              │  CONTINUOUS     │
 RStick move  │   TRACKING      │
 >0.1     ──► │                 │
 B pressed    └──────┬──────────┘
                     │
                     │ LBumper toggle
                     │ or goal lost
                     │
              ┌──────▼──────────┐
              │   AUTO-AIM      │
  Timeout     │  (Spin Down)    │
  (3.0s)  ──► │                 │
              └──────┬──────────┘
                     │
                     │ Timeout
                     ▼
              ┌─────────────┐
              │   MANUAL    │
              └─────────────┘
```

### Autonomous State Machine
```
┌──────────────┐
│    START     │
└──────┬───────┘
       │
       ▼
┌──────────────┐
│ MOVE_TO_SHOOT│ ◄──── Drive to shooting position
└──────┬───────┘
       │ arrived
       ▼
┌──────────────┐          Timeout (3s) ──┐
│  AUTO_AIM    │ ◄──── Acquire goal tag  │
└──────┬───────┘          No valid tag ──┤
       │ tag acquired                     │
       ▼                                  │
┌──────────────┐                         │
│    SHOOT     │ ◄──── Fire ball         │
└──────┬───────┘                         │
       │ ball fed                         │
       ▼                                  │
┌──────────────┐          ◄───────────────┘
│ MOVE_TO_PARK │ ◄──── Drive to observation zone
└──────┬───────┘          (FALLBACK MODE if auto-aim failed)
       │ arrived
       ▼
┌──────────────┐
│     PARK     │ ◄──── Stay parked
└──────┬───────┘
       │ delay (1s)
       ▼
┌──────────────┐
│     DONE     │ ◄──── Match over
└──────────────┘
```

---

*This diagram shows the complete architecture of Commits 3 + 6B + 7*  
*Use this as a reference when debugging or adding new features*
