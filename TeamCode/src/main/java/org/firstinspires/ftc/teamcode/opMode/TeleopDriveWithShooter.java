/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.opMode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;
import java.util.TimerTask;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 *  This was copied from external samples to the 2025-11846 repo's teamcode folder 2025-09-14
 */
@TeleOp(name = "Robot: Robot Relative Mecanum Drive With Shooter", group = "Robot")
public class TeleopDriveWithShooter extends OpMode {
    // This declares the motor to be used for the shooter
    DcMotorEx shooterMotor;

    // This declares the two servos used for indexing
    CRServo leftIndexServo;
    CRServo rightIndexServo;

    public static Follower follower;

    ElapsedTime indexTimer = new ElapsedTime();

    boolean indexOn, rightBumperPressedLast, leftBumperPressedLast, bPressedLast, checkTimer = false;
    double shooterVelocity = 0;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");

        leftIndexServo = hardwareMap.get(CRServo.class, "feedServoL");
        rightIndexServo = hardwareMap.get(CRServo.class, "feedServoR");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //For the servos to work properly, the left index servo must be reversed
        leftIndexServo.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("Telemetry Values");
        telemetry.addData("ShooterVelocity value", shooterVelocity);
        telemetry.addData("Shooter Velocity (RPM)", shooterMotor.getVelocity());
        telemetry.addData("Robot X:", follower.getPose().getX());
        telemetry.addData("Robot Y:", follower.getPose().getY());
        telemetry.addData("Robot Heading:", follower.getPose().getHeading());
        telemetry.addData("Total Heading:", follower.getTotalHeading());
        telemetry.addLine("Gamepad 1:");
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Gamepad 2:");
        telemetry.addLine("Press X to fire a loaded ball");
        telemetry.addLine("Press the left bumper to shoot from close to the goals");
        telemetry.addLine("Press the right bumper to shoot from far to the goals");
        telemetry.addLine("Press B to bring the shooter to a stop");
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
        }

        follower.update();

        runShooterVelocity();

        if (gamepad2.xWasPressed()){
            if(indexOn){
                index(0);
                indexOn = false;
                checkTimer = false;
            }else{
                index(1);
                indexOn = true;
                indexTimer.reset();
                checkTimer = true;
            }
        }

        telemetry.addData("Timer", indexTimer.seconds());
        if (indexTimer.seconds() >= 0.25 && checkTimer) {
            index(0);
            indexOn = false;
            checkTimer = false;
        }

        if(gamepad2.right_bumper){
            if(!rightBumperPressedLast){
                shooterVelocity = 200;
                //DO NOT GO OVER 500!!!
            }else{
                rightBumperPressedLast = false;
            }
        }

        if(gamepad2.left_bumper){
            if(!leftBumperPressedLast){
                shooterVelocity = 170;
                //DO NOT GO OVER 500!!!
            }else{
                leftBumperPressedLast = false;
            }
        }

        if(gamepad2.b){
            if(!bPressedLast){
                shooterVelocity = 0;
            }else{
                bPressedLast = false;
            }
        }

        updateTelemetry(telemetry);
    }

    public void shoot(double power){
        shooterMotor.setPower(power);
    }

    public void runShooterVelocity(){
        shooterMotor.setVelocity(shooterVelocity,AngleUnit.DEGREES);
    }

    public void index(double power){
        leftIndexServo.setPower(power);
        rightIndexServo.setPower(power);
    }
}