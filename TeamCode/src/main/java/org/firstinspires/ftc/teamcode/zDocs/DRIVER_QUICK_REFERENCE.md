# Driver Quick Reference Card
**Wayland M.S. FTC - Competition Controls**

---

## 🎮 GAMEPAD 1 - DRIVER

### **Movement**
- **Left Stick:** Forward/Backward/Strafe (X/Y movement)
- **Right Stick:** Rotate robot (turning)

### **Drive Modes**
- **Normal (Default):** Field-relative drive (forward is always away from driver)
- **Left Bumper HELD:** Robot-relative drive (forward is robot's front)

### **Special**
- **Both Triggers + A:** Reset IMU (if robot drifts)

### **LED Control** *(Robot 22154 only)*
- **Left Bumper (tap):** Green LED
- **Right Bumper (tap):** Purple LED

---

## 🎮 GAMEPAD 2 - OPERATOR

### **Shooter Control**
- **DPAD Left:** Low velocity (close shots)
- **DPAD Right:** High velocity (far shots)
- **B Button:** STOP ALL (emergency)

### **Auto-Aim**
- **Y Button:** Single-shot auto-aim (0.5 sec tracking burst)
- **Left Bumper:** Toggle continuous tracking (lock onto goal)
  - While tracking: Robot rotates to face goal automatically
  - Override: Move right stick to take back control
  - Press again to disable

### **Ball Feed**
- **Right Trigger:** Feed ball into shooter

---

## 📊 TELEMETRY GUIDE

### **What to Watch**
```
=== SHOOTER ===
Mode: [Manual / SINGLE-SHOT / CONTINUOUS / AUTO-AIM]
Target: XXXX RPM  ← What shooter is trying to reach
Actual: XXXX RPM  ← What shooter is actually doing
Distance: XX.X in to Tag X  ← If auto-aiming
```

**Shooter Ready When:** Actual RPM ≈ Target RPM (within ~100)

### **Pose Info** *(for debugging)*
```
Pose: X=XX.X Y=XX.X H=XXX°
```
- X: Distance from blue wall (0) to red wall (144)
- Y: Distance from audience (0) to far side (144)
- H: Robot heading (0° = facing audience)

---

## 🎯 AUTO-AIM TIPS

### **When to Use Single-Shot (Y button)**
- Quick shots from any position
- When robot is already lined up
- When goal tag briefly visible
- Less experienced drivers

### **When to Use Continuous Tracking (LBumper)**
- Moving while shooting
- Maintaining lock on goal
- Strafing around defenders
- More experienced drivers

### **Tracking Indicators**
- **GREEN LED:** Tracking active, good lock
- **PURPLE LED:** Manual control, no tracking
- **Telemetry:** Shows "TRACKING" mode and time remaining

---

## 🚨 TROUBLESHOOTING

### **Robot Won't Rotate During Tracking**
- Check: Is goal tag visible? (Telemetry shows "Tags: X")
- Check: Is camera working? (Telemetry shows "Camera: STREAMING")
- Solution: Move right stick to override, reposition robot

### **Shooter Not Reaching Velocity**
- Check: Battery voltage (should be >12V)
- Check: Actual vs Target RPM (in telemetry)
- Solution: If voltage low, swap battery

### **IMU Drift (Robot Rotates in Field-Relative Mode)**
- Check: Does robot rotate slowly even with sticks centered?
- Solution: Both triggers + A on Gamepad 1 to reset IMU

### **Ball Not Feeding**
- Check: Ball Feed status in telemetry (should show "FEEDING")
- Check: Right trigger pressed enough? (>50%)
- Solution: Try pressing trigger again

---

## 📋 PRE-MATCH CHECKLIST

### **Before Init**
- [ ] Check battery voltage (>12.5V)
- [ ] Verify camera is plugged in
- [ ] Clear any balls from robot
- [ ] Position robot in starting tile

### **During Init_Loop**
1. **Select Robot** (DPAD Up/Down, Press A)
   - TestBot
   - Robot 22154
   - Robot 11846

2. **Select Alliance** (DPAD Up/Down, Press A, B to go back)
   - Red
   - Blue

3. **Select Position** (DPAD Up/Down, Press A, B to go back)
   - Near (audience side)
   - Far (far side)

4. **Ready Screen**
   - Verify: Robot, Alliance, Position all correct (✓ marks)
   - Verify: Starting pose matches your tile
   - Press START to begin!

### **During Match**
- [ ] Watch Actual RPM before shooting
- [ ] Use auto-aim when goal tag visible
- [ ] Monitor battery voltage (in telemetry)
- [ ] Communicate with driver about positioning

---

## 🏆 STRATEGY TIPS

### **Early Match (First 30 seconds)**
- Focus on high-percentage shots
- Use auto-aim to build confidence
- Stay clear of defenders

### **Mid Match (30-90 seconds)**
- Mix auto-aim with manual shooting
- Strafe while tracking for evasion
- Watch for low battery warnings

### **End Match (Last 30 seconds)**
- Maximize shots before time
- Park in observation zone if time low
- Don't risk risky shots

---

## 🎭 ALLIANCE COLORS

**In Telemetry & LEDs:**
- **Green LED:** Our team color (regardless of Red/Blue alliance)
- **Purple LED:** Opponent balls (regardless of Red/Blue alliance)
- **Telemetry:** Shows "Alliance: Red" or "Alliance: Blue"

**On Field:**
- Red Alliance: Starts on red side, shoots into red goal
- Blue Alliance: Starts on blue side, shoots into blue goal
- Code handles mirroring automatically!

---

## 📞 EMERGENCY CONTACTS

**Pit Crew:** [Your pit crew radio channel]  
**Robot Issues:** [Your tech lead]  
**Driver Station Problems:** [FTA assistance]

---

**MOST IMPORTANT:**
1. **B button (GP2) stops everything** - use if something goes wrong!
2. **Communicate!** Driver and operator must talk constantly
3. **Have fun!** This is what we've been preparing for! 🎉

---

*Print this double-sided and laminate for competition!*  
*Good luck, team! 🤖🏆*
