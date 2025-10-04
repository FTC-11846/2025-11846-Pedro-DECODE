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
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
@Disabled  // Add this line to each OpMode
public class TeleopMecanumFieldRelativeDriveWithShooter extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // This declares the motor to be used for the shooter
    DcMotorEx shooterMotor;

    // This declares the two servos used for indexing
    CRServo leftIndexServo;
    CRServo rightIndexServo;

    ElapsedTime indexTimer = new ElapsedTime();

    boolean shooterOn, indexOn, yPressedLast, bPressedLast, rightBumperPressedLast, leftBumperPressedLast, checkTimer = false;
    double shooterVelocity = 0;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lr");
        backRightDrive = hardwareMap.get(DcMotor.class, "rr");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "sm");

        leftIndexServo = hardwareMap.get(CRServo.class, "lis");
        rightIndexServo = hardwareMap.get(CRServo.class, "ris");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //For the servos to work properly, the left index servo must be reversed
        leftIndexServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    public void loop() {
        telemetry.addLine("Telemetry Values");
        telemetry.addData("shooterVelocity value", shooterVelocity);
        telemetry.addData("Shooter Velocity (RPM)", shooterMotor.getVelocity());
        telemetry.addLine("Gamepad 1:");
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("");
        telemetry.addLine("Gamepad 2:");
        //telemetry.addLine("Press A to turn the shooter on and off");
//        telemetry.addLine("Press Y to increase shooter velocity");
//        telemetry.addLine("Press B to decrease shooter velocity");
        telemetry.addLine("Double tap X to fire a loaded ball");
        telemetry.addLine("Press the left bumper to shoot from close to the goals");
        telemetry.addLine("Press the right bumper to shoot from far to the goals");
        telemetry.addLine("");
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        runShooterVelocity();

//        if (gamepad2.aWasPressed()){
//            if(shooterOn){
//                shoot(0);
//                shooterOn = false;
//            }else{
//                shoot(1);
//                shooterOn = true;
//            }
//        }

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

//        if (gamepad2.y){
//            if(!yPressedLast){
//                shooterVelocity += 0.1;
//            }else if (yPressedLast) {
//                yPressedLast = false;
//            }
//
//        }
//
//        if(gamepad2.b){
//            if(!bPressedLast){
//                shooterVelocity -= 0.1;
//            }else if (bPressedLast){
//                bPressedLast = false;
//
//            }
//
//        }

        if(gamepad2.right_bumper){
            if(!rightBumperPressedLast){
                shooterVelocity = 200;
                //DO NOT GO OVER 500!!!
            }else if (rightBumperPressedLast){
                rightBumperPressedLast = false;
            }
        }

        if(gamepad2.left_bumper){
            if(!leftBumperPressedLast){
                shooterVelocity = 170;
                //DO NOT GO OVER 500!!!
            }else if (leftBumperPressedLast){
                leftBumperPressedLast = false;
            }
        }

//        if(gamepad2.yWasPressed()){
//            shooterVelocity += 1;
//        }else if(gamepad2.yWasReleased()){
//            shooterVelocity = 0.1;
//        }
//
//        if(gamepad2.bWasPressed()){
//            shooterVelocity -= 1;
//        }else if(gamepad2.bWasReleased()){
//            shooterVelocity = 0.1;
//        }
//
//        if(gamepad2.a){
//            shooterVelocity = 0;
//        }

    updateTelemetry(telemetry);
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Robot Angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
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

    public void rpmReading(){}
}
