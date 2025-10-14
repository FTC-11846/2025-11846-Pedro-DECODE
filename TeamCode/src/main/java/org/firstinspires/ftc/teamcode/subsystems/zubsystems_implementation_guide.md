# FTC Code Refactoring Implementation Guide
## Wayland M.S. FIRST FTC - Pre-Competition Strategy

---

## Executive Summary

**Recommendation:** Implement clean subsystem architecture NOW (4 days), defer command-based to post-competition.

**Timeline:**
- Days 1-4: Subsystem refactoring
- Days 5-7: Testing and bug fixes  
- Days 8-10: Autonomous development
- Days 11-13: Driver practice and tuning
- Day 14: Feature freeze

**Risk Level:** Low - Subsystems are straightforward and provide immediate benefits

---

## Phase 1: Immediate Implementation (Days 1-4)

### Step 1: Create Subsystem Package (Day 1)

Create new package: `org.firstinspires.ftc.teamcode.subsystems`

Add these files (provided in artifacts):
1. `RobotConfig.java` - Multi-robot configuration enum
2. `ShooterSubsystem.java` - Unified shooter control
3. `VisionSubsystem.java` - AprilTag detection
4. `IndexerSubsystem.java` - Ball feeding system

### Step 2: Migrate Robot-Specific Code (Day 2)

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

### Step 3: Refactor TeleOp OpModes (Days 2-3)

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

### Step 4: Create Autonomous Base Class (Day 3)

```java
public abstract class AutoBase extends OpMode {
    protected ShooterSubsystem shooter;
    protected IndexerSubsystem indexer;
    protected VisionSubsystem vision;
    protected RobotConfig config;
    
    @Override
    public void init() {
        config = getConfig(); // Override in subclass
        shooter = new ShooterSubsystem(hardwareMap, config);
        indexer = new IndexerSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
    }
    
    protected abstract RobotConfig getConfig();
    protected abstract void runAutonomous();
}
```

Then create specific autonomous OpModes:
```java
@Autonomous(name = "Red Near - 22154")
public class RedNear22154 extends AutoBase {
    @Override
    protected RobotConfig getConfig() {
        return RobotConfig.ROBOT_22154;
    }
    
    @Override
    protected void runAutonomous() {
        // Your autonomous logic here
        // Uses subsystems from base class
    }
}
```

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

### Recommended Approach: Single Repository, Configuration-Based

**Structure:**
```
master (main development)
├── 22154-competition (minimal differences)
└── 11846-competition (minimal differences)
```

**What Goes in Branches?**
- OpMode files with robot-specific names
- Hardware configuration files (`.xml`)
- That's it! Everything else in master

**What Goes in RobotConfig?**
- Hardware differences (dual motors, LEDs, etc.)
- Tuning constants (PIDF, ballistics)
- Starting positions
- Feature flags (hasFoldingMechanism, etc.)

**Benefits:**
- ✅ 90% of code shared in master
- ✅ Easy to propagate fixes to all robots
- ✅ Clear documentation of differences
- ✅ Simple to add new robots

**Alternative (NOT Recommended):**
Forking repositories - adds overhead, communication issues, harder to share fixes

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

### 1. LED Subsystem (If Time Permits)

For Robot 22154 with LEDs:
```java
public class LEDSubsystem {
    private Servo ledL, ledR;
    
    public void setColor(LEDColor color) {
        double position = color.getServoPosition();
        ledL.setPosition(position);
        ledR.setPosition(position);
    }
    
    public enum LEDColor {
        GREEN(0.444), PURPLE(0.722);
        private final double position;
        // ...
    }
}
```

### 2. Odometry Subsystem (Future Enhancement)

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

### 3. Emergency Procedures

**If Subsystems Break During Competition:**
1. Keep old TeleOp as "TeleOp - BACKUP"
2. Test backup OpMode before competition
3. Switch in driver station if issues occur

**Emergency Checklist:**
- [ ] Backup OpMode tested and working
- [ ] Paper copy of hardware names
- [ ] Phone with FTC SDK documentation
- [ ] Flashlight and tools

### 4. Code Review Checklist

Before feature freeze:
- [ ] All OpModes compile
- [ ] RobotConfig set correctly for each robot
- [ ] No hardcoded values that should be in config
- [ ] Telemetry shows useful information
- [ ] Emergency stop works (B button)
- [ ] Autonomous doesn't crash
- [ ] Comments explain complex logic

### 5. Git Workflow

**Daily Commits:**
```bash
git add .
git commit -m "Day 2: Added ShooterSubsystem, tested on TestBot"
git push origin master
```

**Branch Management:**
```bash
# Work on master for common code
git checkout master

# Switch to robot branch for specific OpModes
git checkout 22154-competition
git merge master  # Get latest common code
# Add robot-specific OpModes
git push origin 22154-competition
```

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

## Questions to Consider

Before implementing, discuss with team:

1. **Which robot to test on first?** (Recommendation: TestBot)
2. **Who refactors which subsystem?** (Divide work)
3. **What's minimum viable autonomous?** (Just park? Score 1 ball?)
4. **What if we run into major issues Day 7?** (Revert to backup)
5. **Post-competition priorities?** (Commands? New features? Code cleanup?)

---

*Document Version 1.0 - Created October 14, 2025*
*For: Wayland M.S. FTC Team - 2025-2026 DECODE Season*
