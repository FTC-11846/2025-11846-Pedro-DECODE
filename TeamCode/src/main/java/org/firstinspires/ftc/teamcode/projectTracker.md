# Wayland M.S. FIRST FTC Robotics Code Team
## 2025-2026 DECODE Season Project Documentation

> **Last Updated:** October 14, 2025  
> **Team:** Wayland Middle School FTC Team  
> **Season:** 2025-2026 DECODE™ presented by RTX  
> **⚠️ FIRST COMPETITION: October 31, 2025 - Feature Freeze by October 28**

---

# 📋 PROJECT STATUS

## 🚨 CRITICAL TIMELINE

**First Competition:** October 31, 2025
**Feature Freeze:** October 28, 2025 (NLT)
**Driver Practice:** October 28-30 (no code changes)

**Days Remaining:** ~14 days to feature complete code

---

## 🎯 Next Steps (Priority Order)

### URGENT (This Week - Oct 14-20)
1. **CRITICAL: Multi-Robot Code Architecture** - Implement subsystem classes and/or NextFTC command system to support 2 competition robots (22154-RC, 11846-RC) from single codebase
2. **Auto-Aim Heading Correction**: Complete Y button rotation to face goal using AprilTag bearing
3. **Robot-Specific Configurations**: Create configuration system for hardware differences (1 vs 2 shooter motors, different mechanisms)
4. **Basic Autonomous OpModes**: Create competition-ready autonomous for each competition robot

### High Priority (Oct 21-27 - Pre-Competition Week)
1. **Competition Testing**: Full system testing on both competition robots (22154-RC, 11846-RC)
2. **Autonomous Refinement**: Test and tune autonomous routines
3. **Driver Practice Integration**: Ensure TeleOp is stable and driver-friendly
4. **Code Documentation**: Comments and emergency troubleshooting notes
5. **Backup Plans**: Fallback code versions if issues arise

### Post-Competition (Nov 1+)
1. **Competition Debrief**: Analyze what worked and what needs improvement
2. **Feature Expansion**: Add advanced features for next competitions
3. **Code Refactoring**: Clean up technical debt from pre-competition rush
4. **Alliance Coordination Features**: Improve multi-robot cooperation

---

## 🔄 In Progress

### Active Work (Current Week)
- **Multi-robot branch management**: Currently using separate branches (master, 22154, 11846) with customized TeleOps - needs consolidation for 2 competition robots
- **Auto-aim heading correction**: Rotate to face goal using AprilTag bearing
- **Autonomous development**: Creating competition OpModes for 22154-RC and 11846-RC
- **Subsystem architecture**: Deciding between custom classes vs NextFTC command system

### Recently Completed (Past Week)
- ✅ **Panels coordinate display fixed** - Pedro Pathing offset properly initialized
- ✅ **Shooter velocity control** - Refactored from voltage to RPM-based control with PIDF tuning
- ✅ **22154 hardware integration** - LED's, lifters, and basic mechanisms functional
- ✅ **11846 hardware integration** - Drivetrain and folding mechanism operational

---

## 🔧 Current Challenges

### CRITICAL Challenges (Blocking Competition Readiness)
1. **Multi-Robot Code Management**: Two competition robots with different hardware need to share codebase without constant branching/merging
2. **Time Crunch**: 14 days to feature freeze with major architectural decisions pending
3. **Autonomous Development**: Need working autonomous for 2 competition robots in parallel
4. **Hardware Variations**: Significant differences between robots (shooter motors, mechanisms, sensors)

### Technical Challenges
1. **AprilTag Reliability**: Ensuring consistent goal detection at varying distances and angles during competition
2. **Shooter Consistency**: PIDF tuning for reliable scoring across battery levels and distances
3. **Vision Processing**: Implementing reliable motif detection using cameras or sensors
4. **Field Positioning**: Accurate localization and path following during autonomous

### Programming Challenges
1. **Code Organization**: Urgent need for modular subsystem architecture
2. **Team Coordination**: Multiple programmers working on 2 different competition robot branches
3. **Testing Workflow**: Limited time for testing across both competition robots before competition
4. **Feature Parity**: Keeping common features (drive, auto-aim) consistent across competition robots

### Strategic Challenges
1. **Feature Prioritization**: What must work for competition vs what can wait
2. **Game Strategy**: Finalizing autonomous routines and scoring priorities
3. **Alliance Coordination**: Programming for effective cooperation with alliance partners
4. **Backup Plans**: Ensuring robots have fallback code if advanced features fail

---

## ✅ Completed

### Core Systems
- [x] Repository setup and SDK integration
- [x] Basic project structure from DECODE starter template
- [x] Team collaboration workflow established with Git branches
- [x] Field-relative mecanum drive with IMU feedback
- [x] Pedro Pathing 2.0 integration
- [x] Panels dashboard coordinate system (fixed Oct 7)

### Vision & Targeting
- [x] AprilTag vision detection with comprehensive telemetry
- [x] AprilTag-based auto-aim for shooter (distance-based power calculation)
- [x] Continuous AprilTag status display with friendly tag names
- [x] Tunable ballistic equation constants

### Robot Control
- [x] Starting position selection menu for TeleOp
- [x] Shooter motor control (velocity-based with RPM feedback)
- [x] Indexer servo control
- [x] Hardware configuration files for multiple robots

### Robot-Specific Implementations
- [x] **TestBot (Code Team)**: Auto-aim TeleOp verified working
- [x] **22154-RC**: Refactored for shooter PIDF tuning, LED's, lifters operational
- [x] **11846-RC**: Drivetrain and folding mechanism functional

---

## 🤖 Multi-Robot Configuration Status

### Current Branch Strategy
```
master (shooter-by-distance) - Development branch (TestBot for alpha/beta testing)
  ├─ 22154: Competition robot with 2-motor shooter, LED's, lifters
  └─ 11846: Competition robot with folding mechanism
```

**Note:** TestBot is used for internal development and alpha/beta testing of major features only. Focus for competition prep is on 22154-RC and 11846-RC.

### Robot Hardware Differences

| Feature | TestBot (Dev Only) | 22154-RC | 11846-RC |
|---------|-------------------|----------|----------|
| **Purpose** | Alpha/beta testing | Competition | Competition |
| Shooter Motors | 1 | 2 | 1 |
| LED System | No | Yes | TBD |
| Lifter/Elevator | No | Yes | No |
| Folding Mechanism | No | No | Yes |
| Indexer Servos | 2 | 2 (repurposed) | 2 |

### Urgent Need
**Consolidate branches into single codebase with:**
- Subsystem classes for major mechanisms
- Robot-specific configuration files (22154-RC, 11846-RC)
- Shared TeleOp and Autonomous base classes
- Hardware abstraction layer

---

# 📖 PROJECT OVERVIEW

## 🎯 About This Project

This repository contains the robot control code for our FTC team competing in the 2025-2026 **DECODE™** season. We are programming autonomous and driver-controlled robots capable of competing in FIRST Tech Challenge matches involving artifact collection, pattern creation, and strategic scoring.

**Competition Context:**
- **Two Competition Robots**: 22154-RC and 11846-RC
- **Development Platform**: TestBot for alpha/beta testing of major features (not competition-focused)
- **First Competition**: October 31, 2025
- **Code Freeze**: October 28, 2025 for driver practice
- **Development Time**: ~2 weeks to feature-complete code

### Game Summary: DECODE™

**DECODE™** is a game where 2 competing alliances of 2 teams each score purple and green artifacts in their goal, build patterns, and race back to their base before time runs out. The match consists of a 30-second autonomous period followed by a 2-minute driver-controlled period.

**Key Game Elements:**
- **ARTIFACTS**: Purple and green balls (5-inch diameter) that serve as the primary game pieces
- **GOALS**: Alliance-specific scoring locations where artifacts are deposited
- **MOTIFS**: Randomized patterns displayed on the OBELISK that determine scoring bonuses
- **RAMPS**: Areas where robots create patterns based on the match motif
- **BASE**: Starting and ending location for robots

**Scoring Mechanics:**
- Score artifacts in alliance goals to "classify" them
- Build patterns on ramps based on the randomized motif shown at match start
- Earn bonuses for completing motif-based patterns
- Top teams are expected to complete around 20 cycles per match
- Return to base at match end for additional points

---

## 🛠 Technical Stack

### Programming Environment
- **Primary IDE**: Android Studio Ladybug (2024.2) or later
- **Programming Language**: Java
- **SDK**: Official FIRST Tech Challenge SDK 11.0 for DECODE (2025-2026)
- **Hardware Platform**: REV Robotics Control Hub and Expansion Hub
- **Path Following**: Pedro Pathing 2.0
- **Dashboard**: FTC Control Panels for telemetry and field visualization

### Development Approach
- **Version Control**: Git with GitHub for team collaboration
- **Branching Strategy**: Currently using robot-specific branches (master, 22154, 11846)
- **Code Organization**: Transitioning to modular subsystems or NextFTC command system
- **Programming Style**: Following FTC best practices and sample code patterns
- **AI Integration**: Using AI assistance for code generation, debugging, and optimization

### Hardware Configuration

**Common Components (All Robots):**
- REV Control Hub
- GoBilda mecanum drivetrain (4 DC motors)
- GoBilda Pinpoint odometry system
- Webcam for AprilTag detection
- IMU for field-relative driving
- Dual servo indexer

**Robot-Specific Hardware:**
- **TestBot**: Basic shooter (1 motor), development platform
- **22154-RC**: Dual shooter motors, LED system, lifting mechanism
- **11846-RC**: Single shooter motor, folding mechanism

---

## 🏆 Season Goals

### Competition Performance Goals
- **First Competition (Oct 31)**: Functional autonomous and reliable TeleOp for all robots
- **Qualifying Events**: Consistently score in top 50% of teams at local qualifiers
- **Regional Advancement**: Qualify for regional/state competition
- **Technical Excellence**: Achieve reliable autonomous operation and efficient scoring
- **Team Collaboration**: Effective alliance partnership and strategic play

### Learning and Development Goals
- **Programming Skills**: Advanced Java programming and robotics-specific techniques
- **Project Management**: Experience with version control, team coding, and tight deadlines
- **Problem Solving**: Debug complex hardware-software integration issues under time pressure
- **Competition Experience**: Multiple competition events and performance under pressure

### Technical Achievements
- **Multi-Robot Support**: Single codebase supporting 3 different robot configurations
- **Autonomous Scoring**: Reliable autonomous artifact scoring and pattern creation
- **Vision-Based Targeting**: AprilTag-based auto-aim for consistent scoring
- **Code Quality**: Maintainable, modular, well-documented codebase despite time constraints

---



## 💡 AI Development Guidelines

When using AI assistance for this project, please provide this context:

**Robot Type**: FTC competition robots for DECODE 2025-2026 season  
**Hardware**: REV Control Hub, GoBilda mecanum chassis with Pinpoint odometry, various mechanisms  
**Programming**: Java in Android Studio using FTC SDK 11.0 with Pedro Pathing 2.0  
**Game Objectives**: Score artifacts, build patterns based on motifs, return to base  
**Competition Level**: Middle school FTC team with 2 competition robots (22154-RC, 11846-RC) plus TestBot for development  
**Timeline**: First competition October 31, 2025 - URGENT development timeline

**Current Implementation Status:**
- ✅ **Field-relative mecanum drive**: Functional with IMU feedback on all robots
- ✅ **AprilTag vision detection**: Working with comprehensive telemetry output
- ✅ **Shooter control system**: Velocity-based with PIDF tuning
- ✅ **Auto-aim distance calculation**: Tunable ballistic equation (Y button on gamepad2)
- ✅ **Starting position selection**: Interactive menu with 4 alliance positions
- ✅ **Panels coordinate display**: Pedro Pathing offset properly initialized
- 🔄 **Auto-aim heading correction**: In progress - needs bearing-based rotation
- 🔄 **Multi-robot architecture**: URGENT - needs subsystem/command system
- 🔄 **Autonomous OpModes**: In development for competition

**Active Development Focus (Pre-Competition):**
- **CRITICAL**: Multi-robot code architecture (subsystems or NextFTC) for 22154-RC and 11846-RC
- Auto-aim heading adjustment using AprilTag bearing
- Competition-ready autonomous routines for both competition robots
- Robot-specific configuration management
- Testing and validation on 22154-RC and 11846-RC

**Known Working Code Patterns:**
- Use sample-based approach (copy from FTC samples, then modify)
- Mecanum drive calculations with power normalization
- AprilTag detection with vision portal management
- Field-relative coordinate transformation using IMU
- Distance-based auto-aim with tunable constants
- Velocity-based shooter control with voltage-scaled F term

**Current Shooter Auto-Aim System:**
- Goal detection: Blue (tag 20), Red (tag 24)
- Distance reading: `detection.ftcPose.range` in inches
- Velocity control: PIDF with voltage-compensated F term
- Control mode: RUN_USING_ENCODER with RPM telemetry
- Button: Y (gamepad2) triggers auto-aim
- Next feature: Heading correction using `detection.ftcPose.bearing`

**Multi-Robot Configuration Notes:**
- **TestBot**: 1 shooter motor, used for alpha/beta development only - not competition-focused
- **22154-RC**: 2 shooter motors, LED system, lifters, modified indexer servos - COMPETITION
- **11846-RC**: 1 shooter motor, folding mechanism - COMPETITION
- Currently using separate branches - needs consolidation ASAP for competition robots

**Testing Status:** 
- TestBot: Used for alpha/beta feature testing (auto-aim distance calculation verified)
- 22154-RC: Basic hardware operational, PIDF tuning in progress - COMPETITION READY TARGET
- 11846-RC: Drivetrain and folding mechanism functional - COMPETITION READY TARGET

**Competition Readiness Checklist:**
- [ ] Multi-robot code architecture implemented
- [ ] Autonomous OpModes created and tested for each robot
- [ ] Auto-aim heading correction functional
- [ ] Driver practice completed with stable code
- [ ] Backup code versions prepared
- [ ] Emergency troubleshooting documentation

---

### Key Resources
- [Official FTC Programming Resources](https://www.firstinspires.org/resource-library/ftc/technology-information-and-resources)
- [FTC SDK Documentation](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
- [Pedro Pathing Documentation](https://pedropathing.com/docs)
- [NextFTC Documentation](https://nextftc.dev/) - Under evaluation for command system
- [Game Manual and Q&A](https://www.firstinspires.org/resource-library/ftc/game-and-season-info)
- [REV Robotics Documentation](https://docs.revrobotics.com/)

---

*This document is maintained by the Wayland M.S. FTC programming team and updated regularly to reflect current project status and goals.*

**Last Major Updates:**
- Oct 14, 2025: Added competition timeline, multi-robot status, urgency priorities
- Oct 12, 2025: Merged shooter-by-distance with auto-aim and coordinate fixes
- Oct 7, 2025: Fixed Panels coordinate display issue
- Oct 5, 2025: Added starting position selection menu