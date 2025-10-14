# Main Character Subsystems - Quick Reference Card

## 🎮 TeleOp Controls (Gamepad Layout)

### Gamepad 1 - Drive & LED
```
LEFT STICK: Drive (X/Y)
RIGHT STICK: Rotate
A (hold): Robot-relative mode
A (release): Field-relative mode  
LEFT BUMPER: Set LED Green (22154 only)
RIGHT BUMPER: Set LED Purple (22154 only)
LEFT TRIGGER + RIGHT TRIGGER + A: Reset IMU
```

### Gamepad 2 - Shooter & Ball Feed
```
LEFT BUMPER: Low shooter velocity
RIGHT BUMPER: High shooter velocity
B: Stop shooter
Y: Auto-aim (detect goal tag)
X: Feed ball
A: Reverse ball feed (unjam)
```

---

## 🤖 Runtime Robot Selection

### Init Sequence (Before START)
1. **Select Robot:**
   - DPAD UP/DOWN: Navigate
   - A: Confirm (TestBot / Robot 22154 / Robot 11846)

2. **Select Starting Position:**
   - DPAD UP/DOWN: Navigate  
   - A: Confirm (Red Near / Red Far / Blue Near / Blue Far)

3. **Press START** to begin TeleOp

---

## 📦 Subsystem Quick Reference

### Shooter
```java
shooter.setLowVelocity();
shooter.setHighVelocity();
shooter.setVelocityRPM(3500);
shooter.setAutoAimVelocity(distance, tagId);
shooter.stop();
shooter.periodic(); // Call in loop()!

// Telemetry
shooter.getTargetVelocityRPM();
shooter.getActualVelocityRPM();
shooter.isAutoAimActive();
```

### BallFeed
```java
ballFeed.startFeed();
ballFeed.startFeed(0.5); // Custom duration
ballFeed.startReverse();
ballFeed.stopFeed();
ballFeed.periodic(); // Call in loop()!

// Status
ballFeed.isFeeding();
ballFeed.getRemainingFeedTime();
```

### Vision
```java
vision.getDetections(); // All tags
vision.findGoalTag(); // Blue or Red goal
vision.findMotifTag(tagId);
vision.hasDetections();

// Auto-aim
Vision.AutoAimResult result = vision.getAutoAimData();
if (result.success) {
    shooter.setAutoAimVelocity(result.distanceInches, result.tagId);
}

// Utility
Vision.getTagFriendlyName(20); // "Blue Goal"
```

### LED (Robot 22154 only)
```java
if (led != null) {
    led.setGreen();
    led.setPurple();
    led.setRed();
    led.setBlue();
    led.setYellow();
    led.setWhite();
    led.turnOff();
    led.setAllianceColor(isGreen);
}
```

---

## 📊 Tunable Constants (via Panels Dashboard)

### Shooter.Constants
- `LOW_VELOCITY_RPM` - Low power preset
- `HIGH_VELOCITY_RPM_22154` - High power for 22154
- `HIGH_VELOCITY_RPM_11846` - High power for 11846
- `BASELINE_POWER_22154` - Auto-aim starting power
- `LINEAR_CORRECTION_FACTOR` - RPM per inch distance
- `PIDF_P_22154` - Proportional gain
- `NOMINAL_VOLTAGE` - Voltage compensation reference

### BallFeed.Constants
- `DEFAULT_FEED_DURATION` - Seconds to feed
- `FEED_POWER` - Forward power
- `REVERSE_POWER` - Reverse power (unjam)

### Vision.Constants
- `BLUE_GOAL_TAG_ID` - Blue goal AprilTag (20)
- `RED_GOAL_TAG_ID` - Red goal AprilTag (24)
- `DECIMATION` - Vision processing speed

### LED.Constants
- `GREEN` - Green position (0.444)
- `PURPLE` - Purple position (0.722)
- `RED`, `BLUE`, `YELLOW`, `WHITE` - Other colors

---

## 🔧 MainCharacter Configuration

### Adding Hardware Name
```java
public String getNewMotorName() {
    switch (this) {
        case ROBOT_22154: return "motor22154";
        case ROBOT_11846: return "motor11846";
        case TEST_BOT:
        default: return "motorTest";
    }
}
```

### Adding Feature Flag
```java
public boolean hasNewFeature() {
    return this == ROBOT_22154;
}
```

### Adding New Robot
```java
public enum MainCharacter {
    TEST_BOT,
    ROBOT_22154,
    ROBOT_11846,
    ROBOT_NEW; // Just add here!
    
    // Then add its config in switch statements
}
```

---

## ⚠️ Common Mistakes to Avoid

### ❌ DON'T FORGET
```java
@Override
public void loop() {
    shooter.periodic();  // REQUIRED!
    ballFeed.periodic(); // REQUIRED!
    
    // ... your control code
}
```

### ❌ DON'T HARDCODE
```java
// Bad
if (robotName.equals("22154")) {
    shooterVelocity = 4000;
}

// Good
shooter.setHighVelocity(); // Uses character-specific constant
```

### ❌ DON'T ASSUME HARDWARE EXISTS
```java
// Bad
led.setGreen(); // Crashes on TestBot!

// Good
if (led != null) {
    led.setGreen();
}
```

### ❌ DON'T SKIP INITIALIZATION
```java
// In init() AFTER robot selected
shooter = new Shooter(hardwareMap, character);
ballFeed = new BallFeed(hardwareMap, character);
vision = new Vision(hardwareMap);
if (character.hasLEDSystem()) {
    led = new LED(hardwareMap, character);
}
```

---

## 🚨 Emergency Procedures

### Competition Day Quick Fixes
1. **Shooter too fast/slow:**
   - Open Panels Dashboard
   - Adjust `Shooter.Constants.HIGH_VELOCITY_RPM_22154`
   - Press refresh

2. **Auto-aim inaccurate:**
   - Adjust `Shooter.Constants.BASELINE_POWER_22154`
   - Adjust `Shooter.Constants.LINEAR_CORRECTION_FACTOR`

3. **Ball feed timing wrong:**
   - Adjust `BallFeed.Constants.DEFAULT_FEED_DURATION`

4. **OpMode crashes:**
   - Switch to "TeleOp - BACKUP OLD"
   - Check hardware names in exception message

### Pre-Match Checklist
- [ ] Select correct robot in init_loop
- [ ] Select correct starting position
- [ ] Verify camera stream shows tags
- [ ] Test shooter at low power first
- [ ] Test ball feed
- [ ] Test emergency stop (Gamepad2 B)

---

## 📞 Help Resources

**FTC Discord:** https://discord.gg/first-tech-challenge
**r/FTC Reddit:** https://reddit.com/r/FTC
**Pedro Pathing Docs:** https://pedropathing.com/docs
**Official FTC Docs:** https://ftc-docs.firstinspires.org

---

*Main Character Energy - Because Every Robot Deserves to be the Star! 🌟*
*Updated: October 14, 2025*
