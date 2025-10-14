# FTC Code Refactoring Implementation Guide
## Wayland M.S. FIRST FTC - Pre-Competition Strategy
### "Main Character Energy" - Every Robot Deserves to be the Star! 🌟

---

## Executive Summary

**Recommendation:** Implement clean subsystem architecture NOW (4 days), defer command-based to post-competition.

**Timeline:**
- Days 1-4: Subsystem refactoring ("Main Character" package)
- Days 5-7: Testing and bug fixes
- Days 8-10: Autonomous development
- Days 11-13: Driver practice and tuning
- Day 14: Feature freeze

**Risk Level:** Low - Subsystems are straightforward and provide immediate benefits

**Key Innovation:** Runtime robot selection eliminates need for separate OpModes per robot!

---

## Phase 1: Immediate Implementation (Days 1-4)

### Step 1: Create Subsystem Package (Day 1)

Create new package: `org.firstinspires.ftc.teamcode.subsystems`

Add these files (provided in artifacts):
1. `MainCharacter.java` - Multi-robot configuration enum (formerly RobotConfig)
2. `Shooter.java` - Unified shooter control (supports single/dual motors)
3. `Vision.java` - AprilTag detection and auto-aim
4. `BallFeed.java` - Ball feeding system (supports single/dual/independent motors)
5. `LED.java` - RGB indicator lights (Robot 22154 only)

**Key Features:**
- All classes use `@Configurable` inner Constants classes for Panels dashboard
- Naming convention: Drop "Subsystem" suffix (package name indicates purpose)
- Runtime robot selection via driver station (no separate OpModes needed!)

### Step 2: Runtime Robot Selection (GAME CHANGER!)

**No more separate OpModes for each robot!** The new TeleOp uses a two-stage selection:

**Stage 1: Select Robot**
- During init_loop, use DPAD UP/DOWN to select robot
- Press A to confirm
- Options: TestBot, Robot 22154, Robot 11846

**Stage 2: Select Starting Position**
- Use DPAD UP/DOWN to select starting position
- Press A to confirm
- Options: Red Near, Red Far, Blue Near, Blue Far

**Benefits:**
- ✅ Single TeleOp OpMode for all robots
- ✅ Quick robot switching for testing
- ✅ Driver station selection (no code changes!)
- ✅ Eliminates OpMode proliferation

**Implementation:**
```java
// The TeleOp automatically handles this in init_loop
// Just select on driver station before pressing START
```

**Current Problem:** Branch22154 and Branch11846 have duplicate code with minor differences

**Solution:** Use `RobotConfig.ACTIVE_ROBOT` to handle differences

**In your OpMode:**
```java
// Set at top of init()
RobotConfig config = RobotConfig.ROBOT_22154; // or ROBOT_11846

// Initialize subsystems with config
shooter = new ShooterSubsystem(hardwareMap, config);
```

**Benefits:**
- ✅ Single codebase for all robots
- ✅ No more branch merging nightmares
- ✅ Easy to add new robots
- ✅ Clear documentation of differences

### Step 3: Constants Structure for Panels Dashboard

**Each subsystem has its own `@Configurable` inner Constants class:**

```java
public class Shooter {
    @Configurable
    public static class Constants {
        public static double LOW_VELOCITY_RPM = 1500;
        public static double HIGH_VELOCITY_RPM_22154 = 4000;
        // ... more constants
    }
    // ... rest of class
}
```

**Why this approach?**
- ✅ Constants stay with related subsystem code
- ✅ Auto-appear in Panels dashboard (via @Configurable)
- ✅ Clear organization (Shooter.Constants, Vision.Constants, etc.)
- ✅ No giant monolithic Constants.java file

**Panels Dashboard Structure:**
```
Shooter.Constants
  ├── LOW_VELOCITY_RPM
  ├── HIGH_VELOCITY_RPM_22154
  └── PIDF_P_22154

BallFeed.Constants
  ├── DEFAULT_FEED_DURATION
  └── FEED_POWER

Vision.Constants
  ├── BLUE_GOAL_TAG_ID
  └── DECIMATION
```

**Alternative Considered:** Single SubsystemConstants.java file
- ❌ Harder to maintain
- ❌ Constants separated from implementation
- ❌ Not recommended for this project

Follow the pattern in `TeleopDriveSubsystems.java`:

**Old way (in OpMode):**
```java
shooterMotorL.setVelocity(targetVelocity);
if (shooterMotorR != null) {
    shooterMotorR.setVelocity(targetVelocity);
}
```

**New way (using subsystem):**
```java
shooter.setHighVelocity();
// Subsystem handles single vs dual motors automatically
```

**Migration Checklist:**
- [ ] Create subsystem instances in `init()`
- [ ] Call subsystem methods instead of direct hardware
- [ ] Call `subsystem.periodic()` in `loop()`
- [ ] Remove duplicate hardware initialization code
- [ ] Test on each robot

### Step 4: Using Subsystems in TeleOp (Day 2)

**Old way (in OpMode):**
```java
// Separate handling for single vs dual motors
shooterMotorL.setVelocity(targetVelocity);
if (character == MainCharacter.ROBOT_22154) {
    shooterMotorR.setVelocity(targetVelocity);
}

// LED control only for 22154
if (character.hasLEDSystem()) {
    ballColorLEDL.setPosition(0.444);
    ballColorLEDR.setPosition(0.444);
}
```

**New way (using subsystems):**
```java
// Automatically handles single vs dual motors
shooter.setHighVelocity();

// LED subsystem only exists if robot has LEDs
if (led != null) {
    led.setGreen();
}
```

**Migration Checklist:**
- [x] Create subsystem package and classes
- [x] Add runtime robot selection to TeleOp
- [ ] Test robot selection on driver station
- [ ] Test subsystems on TestBot first
- [ ] Test on Robot 22154 (dual shooters, LEDs)
- [ ] Test on Robot 11846 (single shooter)
- [ ] Verify all gamepad controls work
- [ ] Verify Panels dashboard shows all constants

### Step 5: Create Autonomous Base Class (Day 3)

Since TeleOp now has runtime selection, autonomous can use the same pattern:

```java
@Autonomous(name = "Main Character Auto Red Near")
public class AutoRedNear extends OpMode {
    private Shooter shooter;
    private BallFeed ballFeed;
    private Vision vision;
    private MainCharacter character;
    
    // Stage 1: Select robot in init_loop (same as TeleOp)
    // Stage 2: Run autonomous
    
    @Override
    public void init() {
        // Subsystems initialized after robot selected
    }
    
    @Override
    public void loop() {
        // Your autonomous logic here
        // All subsystems available!
    }
}
```

**Alternative:** Create separate autonomous files per starting position, but ALL use runtime robot selection.

---

## Phase 2: Testing & Validation (Days 5-7)

### Testing Protocol

**Day 5: Basic Functionality**
- [ ] Drive system works on all robots
- [ ] Shooter velocity control accurate
- [ ] Indexer feeds correctly
- [ ] Vision detects tags reliably

**Day 6: Integration Testing**
- [ ] Auto-aim works from multiple distances
- [ ] Robot switching works (change ACTIVE_ROBOT)
- [ ] All gamepad controls respond correctly
- [ ] Telemetry displays accurate data

**Day 7: Stress Testing**
- [ ] Full match simulation (2:30)
- [ ] Battery voltage compensation works
- [ ] No crashes or freezes
- [ ] Emergency stop works

### Bug Fix Priorities
1. **Critical:** Robot doesn't move / crashes
2. **High:** Shooter/auto-aim broken
3. **Medium:** Telemetry issues
4. **Low:** Cosmetic problems

---

## Phase 3: Competition Readiness (Days 8-14)

### Days 8-10: Autonomous Development
- Build 2-3 reliable autonomous routines per robot
- Test autonomous → TeleOp transitions
- Add backup "safe" autonomous (just park)

### Days 11-13: Driver Practice
- **NO CODE CHANGES** during this period
- Drivers practice with frozen code
- Note issues for post-competition fixes

### Day 14: Feature Freeze & Pre-Competition
- Final code review
- Create backup APK
- Print emergency troubleshooting guide
- Charge all batteries

---

## Multi-Robot Management Strategy

### New Approach: Runtime Selection + Single Codebase

**Structure:**
```
master (everything lives here now!)
├── subsystems/
│   ├── MainCharacter.java (robot config)
│   ├── Shooter.java
│   ├── BallFeed.java
│   ├── Vision.java
│   └── LED.java
├── opMode/
│   ├── TeleopDriveSubsystems.java (works for ALL robots)
│   ├── AutoRedNear.java (works for ALL robots)
│   └── AutoBlueFar.java (works for ALL robots)
```

**What Goes in Branches?**
- Hardware configuration XML files only
- That's it!

**What Goes in MainCharacter.java?**
- Hardware names (motor/servo names)
- Hardware capabilities (hasDualShooters, hasLEDs, etc.)
- Tuning constants per robot (PIDF, ballistics)
- Starting pose defaults
- Feature flags

**Benefits:**
- ✅ 100% of code shared in master
- ✅ No branch merging ever
- ✅ Select robot on driver station before start
- ✅ One TeleOp for all robots
- ✅ One set of autonomous OpModes for all robots
- ✅ Easy to add new robots (just add to enum)

**Example: Adding a New Robot**
```java
// In MainCharacter.java
public enum MainCharacter {
    TEST_BOT,
    ROBOT_22154,
    ROBOT_11846,
    ROBOT_NEW; // Add new robot here
    
    // Add its configuration
    public String getShooterMotorLName() {
        switch (this) {
            case ROBOT_NEW: return "newShooterMotor";
            // ...
        }
    }
}
```

That's it! No new branches, no new OpModes!

### Old Approach (NOT RECOMMENDED)

~~Separate branches for each robot with duplicated code~~
~~Forking repositories~~

These approaches create maintenance nightmares and merge conflicts.

---

## Command-Based: Post-Competition Strategy

### Why Wait?

**Time vs Risk:**
- Subsystems: 4 days, low risk ✅
- Commands: 7-10 days, medium risk ⚠️
- Combined: 10-14 days, high risk ❌

**Post-Competition Benefits:**
- No pressure of competition deadline
- Can test thoroughly
- Learn from competition experience
- Implement only features you need

### Migration Path to NextFTC

Your subsystems are already compatible! NextFTC commands wrap subsystems:

**Current (subsystem-based):**
```java
if (gamepad2.y) {
    shooter.setAutoAimVelocity(distance, tagId);
}
```

**Future (command-based):**
```java
gamepad2.y().onTrue(new AutoAimCommand(shooter, vision));
```

**Steps:**
1. Keep existing subsystems (no changes needed)
2. Add NextFTC dependency
3. Create command classes
4. Migrate OpModes incrementally
5. Test each command before moving to next

---

## Additional Recommendations

### 1. LED Subsystem (ALREADY INCLUDED! ✅)

For Robot 22154 with GoBilda RGB LEDs:
```java
// In TeleOp
if (led != null) {  // Only Robot 22154 has LEDs
    led.setGreen();    // Gamepad1 Left Bumper
    led.setPurple();   // Gamepad1 Right Bumper
    led.setAllianceColor(isGreen);  // Smart alliance color
}
```

**Available Colors:** Green, Purple, Red, Blue, Yellow, White, Off

**Tunable via Panels:**
```java
LED.Constants.GREEN = 0.444;
LED.Constants.PURPLE = 0.722;
// etc.
```

### 2. BallFeed Modes (FLEXIBLE ARCHITECTURE ✅)

Supports three modes via MainCharacter config:
- **SINGLE**: One motor only
- **DUAL_SYNCHRONIZED**: Two motors, same power (current default)
- **DUAL_INDEPENDENT**: Two motors, independent control (future)

To switch modes:
```java
// In MainCharacter.java
public BallFeedMode getBallFeedMode() {
    switch (this) {
        case ROBOT_SPECIAL:
            return BallFeedMode.DUAL_INDEPENDENT;
        default:
            return BallFeedMode.DUAL_SYNCHRONIZED;
    }
}
```

### 3. Naming Convention Decisions

**Current Naming:**
- Package: `subsystems` (clear purpose)
- Classes: `Shooter`, `BallFeed`, `Vision`, `LED` (short, clean)
- Config: `MainCharacter` (fun team branding!)

**Why drop "Subsystem" suffix?**
- ✅ Package name already indicates purpose
- ✅ Shorter class names in code
- ✅ Less visual clutter
- ✅ Matches industry patterns (e.g., WPILib)

**Full class names only appear once:**
```java
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

// Then just use:
Shooter shooter = new Shooter(hardwareMap, character);
```

Wrap Pedro Pathing follower in subsystem:
```java
public class DriveSubsystem {
    private final Follower follower;
    
    public void teleopDrive(double x, double y, double heading) {
        follower.setTeleOpDrive(x, y, heading, true);
    }
    
    public Pose getPose() {
        return follower.getPose();
    }
}
```

### 4. Robot 22154 Enhancements Captured ✅

All features from the 22154 branch are now in the unified TeleOp:

**LED Control:**
- ✅ Gamepad1 Left Bumper: Green
- ✅ Gamepad1 Right Bumper: Purple
- ✅ Tunable colors via LED.Constants
- ✅ Only active when robot has LED hardware

**Drive Control:**
- ✅ Gamepad1.A held: Robot-relative mode
- ✅ Gamepad1.A released: Field-relative mode
- ✅ Both triggers + A: Reset IMU yaw

**Shooter Improvements:**
- ✅ Voltage-compensated PIDF
- ✅ Dual motor support (22154 only)
- ✅ Per-robot PIDF tuning constants
- ✅ Debug mode for testing without encoders

**Starting Pose:**
- ✅ Different headings per robot (22154 uses 270°)
- ✅ Configurable via MainCharacter

### 5. Emergency Procedures

**If Subsystems Break During Competition:**
1. Keep old TeleOp as "TeleOp - BACKUP"
2. Test backup OpMode before competition
3. Switch in driver station if issues occur

### 5. Emergency Procedures

**If Subsystems Break During Competition:**
1. Keep old TeleOp as "TeleOp - BACKUP OLD"
2. Test backup before competition
3. Select backup OpMode in driver station if issues occur

**Emergency Checklist:**
- [ ] Backup OpMode tested on all robots
- [ ] Know how to switch OpModes quickly
- [ ] Paper copy of hardware names (for quick fixes)
- [ ] Phone with FTC SDK documentation
- [ ] Flashlight and tools
- [ ] Know which constants can be changed safely

**Quick Fixes at Competition:**
```java
// If shooter RPM needs adjustment
Shooter.Constants.HIGH_VELOCITY_RPM_22154 = 3800; // Tunable via Panels!

// If auto-aim is off
Shooter.Constants.BASELINE_POWER_22154 = 2200;
Shooter.Constants.LINEAR_CORRECTION_FACTOR = 20;

// If ball feed is too fast/slow
BallFeed.Constants.DEFAULT_FEED_DURATION = 0.30;
```

### 6. Code Review Checklist

### 6. Code Review Checklist

Before feature freeze:
- [ ] All OpModes compile
- [ ] TeleOp works on all 3 robots via runtime selection
- [ ] All constants visible in Panels dashboard
- [ ] MainCharacter enum has correct hardware names
- [ ] No hardcoded values that should be in Constants
- [ ] Telemetry shows useful information
- [ ] Emergency stop works (B button on gamepad2)
- [ ] Autonomous OpModes compile
- [ ] LED subsystem only initializes on robots with LEDs
- [ ] BallFeed works in all modes
- [ ] Comments explain complex logic
- [ ] All subsystems have periodic() called in loop()

**Specific Tests Per Robot:**
- [ ] TestBot: Single shooter, no LEDs
- [ ] Robot 22154: Dual shooters, LEDs, lifters
- [ ] Robot 11846: Single shooter, folding mechanism

### 5. Git Workflow (SIMPLIFIED!)

**Daily Commits (Everything on Master):**
```bash
git add .
git commit -m "Day 2: Added Shooter subsystem, tested on all robots"
git push origin master
```

**That's it!** No branch management needed since everything lives in master.

**Testing Different Robots:**
```bash
# Just push to master
git push

# Test on any robot by selecting it on driver station
# No code changes required!
```

**Benefits:**
- No branches to manage
- No merging conflicts
- One source of truth
- Simpler workflow for team

**Hardware Config Files (Optional Branches):**
If you have different hardware XML configurations:
```bash
# Create minimal branches just for XML files
git checkout -b 22154-config
# Add only: FtcRobotController/teamcode/xml/Robot22154.xml
git push origin 22154-config
```

But code? All on master!

---

## Success Metrics

### Before Competition (Oct 28)
- [ ] All 3 robots compile and run
- [ ] TeleOp OpModes work on all robots
- [ ] 2+ autonomous routines per robot
- [ ] Drivers comfortable with controls
- [ ] Auto-aim works reliably
- [ ] Zero crashes during testing

### Competition Day (Oct 31)
- [ ] Robot passes inspection
- [ ] All alliance partners briefed
- [ ] Backup code ready
- [ ] Team knows emergency procedures

### Post-Competition Goals
- [ ] Analyze what worked/didn't work
- [ ] Implement NextFTC commands
- [ ] Add advanced features
- [ ] Refactor based on lessons learned

---

## Time-Saving Tips

1. **Pair Programming:** Two people on one robot = faster + fewer bugs
2. **Test Early:** Don't wait until Day 4 to test on real robot
3. **Use Dashboard:** FTC Dashboard makes tuning MUCH faster
4. **Copy-Paste Smart:** Use subsystem examples as templates
5. **Ask for Help:** FTC Discord, Reddit (/r/FTC) very helpful

---

## Final Thoughts

**This is achievable!** The subsystem refactoring is straightforward and gives you:
- ✅ Cleaner code
- ✅ Multi-robot support
- ✅ Faster development
- ✅ Better competition readiness

**Remember:** Perfect is the enemy of good. Get something working, test it thoroughly, then iterate. You have 14 days - use them wisely!

**Good luck at competition! 🤖🏆**

---

## Summary of Key Decisions

### ✅ What We Implemented

1. **"Main Character Energy"** - Fun, motivating name for robot config
2. **Runtime Robot Selection** - Select robot on driver station (no separate OpModes!)
3. **Dropped "Subsystem" suffix** - Cleaner naming (Shooter, BallFeed, Vision, LED)
4. **@Configurable Constants** - Each subsystem has its own inner Constants class
5. **BallFeed Flexibility** - Supports single, dual-synchronized, or dual-independent modes
6. **LED Subsystem** - Full RGB control with GoBilda PWM LEDs
7. **All 22154 Features** - LED controls, voltage-compensated PIDF, etc.
8. **100% Code in Master** - No branches needed for code

### 🎯 What This Solves

**Your Problem #1: Code Modularity**
- ✅ All constants in subsystem-specific inner classes
- ✅ Visible in Panels dashboard via @Configurable
- ✅ Clear organization (Shooter.Constants, Vision.Constants)

**Your Problem #2: Multi-Robot Strategy**
- ✅ Runtime selection eliminates OpMode explosion
- ✅ All code in master branch
- ✅ Hardware XML files can be in optional branches

**Your Problem #3: Command-Based Timing**
- ✅ Deferred to post-competition (low risk)
- ✅ Current subsystems are command-compatible
- ✅ Easy migration path later

### 📋 Implementation Roadmap

**Day 1: Create Package**
- Create subsystems package
- Add all 5 subsystem files
- Compile and verify

**Day 2: Test on TestBot**
- Run TeleOp, select TestBot
- Test all controls
- Verify Panels dashboard shows constants
- Fix any issues

**Day 3: Test on Competition Robots**
- Test on Robot 22154 (dual shooters, LEDs)
- Test on Robot 11846 (folding mechanism)
- Verify runtime robot selection works
- Fine-tune constants

**Day 4: Polish & Document**
- Add comments
- Create backup OpMode
- Document emergency procedures
- Prepare for autonomous development

**Days 5-7: Testing**
- Stress testing
- Full match simulations
- Battery voltage testing
- Bug fixes

**Days 8-10: Autonomous**
- Create autonomous OpModes
- Use same subsystems
- Use same runtime selection
- Test transitions

**Days 11-13: CODE FREEZE**
- Driver practice only
- No code changes
- Note issues for post-competition

**Day 14: Competition Prep**
- Final checks
- Backup APK
- Charge batteries
- Print emergency guide

---

## Questions Answered

**Q: Why MainCharacter instead of RobotConfig?**
A: Team morale! "Main Character Energy" is fun and memorable. Technical names are boring.

**Q: Why drop "Subsystem" suffix?**
A: Package name already says "subsystems". Shorter names are cleaner. Full name only used once at import.

**Q: Constants in each subsystem or central file?**
A: Each subsystem! Keeps related code together. @Configurable makes them appear in Panels. Easier to maintain.

**Q: Can we support different BallFeed configurations?**
A: Yes! BallFeedMode enum supports: SINGLE, DUAL_SYNCHRONIZED, DUAL_INDEPENDENT.

**Q: How do we select robot without separate OpModes?**
A: Two-stage selection in init_loop: (1) Select robot, (2) Select starting position. All via driver station!

**Q: Did we capture 22154 branch enhancements?**
A: Yes! LED controls, voltage compensation, robot-relative drive toggle, all included.

---

Before implementing, discuss with team:

1. **Which robot to test on first?** (Recommendation: TestBot)
2. **Who refactors which subsystem?** (Divide work)
3. **What's minimum viable autonomous?** (Just park? Score 1 ball?)
4. **What if we run into major issues Day 7?** (Revert to backup)
5. **Post-competition priorities?** (Commands? New features? Code cleanup?)

---

*Document Version 1.0 - Created October 14, 2025*
*For: Wayland M.S. FTC Team - 2025-2026 DECODE Season*