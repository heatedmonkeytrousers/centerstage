/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.A;
import org.checkerframework.checker.units.qual.C;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")

public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;
    private DcMotor shoulderDrive = null;
    private DcMotor tailDrive = null;
    private Servo wristServo = null;
    private Servo clawServo1 = null;
    private Servo clawServo2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive"); //ch3
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); //ch2
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftDrive"); //ch1
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRightDrive"); //ch0
        armDrive1 = hardwareMap.get(DcMotor.class, "armDrive1"); //ch1 expansion hub Motor
        armDrive2 = hardwareMap.get(DcMotor.class, "armDrive2"); //ch2 expansion hub Motor
        shoulderDrive = hardwareMap.get(DcMotor.class, "shoulderDrive"); //ch0 Motor
        wristServo = hardwareMap.get(Servo.class, "wrist"); //ch0 expansion hub Servo
        clawServo1 = hardwareMap.get(Servo.class, "clawServo1"); //ch1 expansion hub Servo
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2"); //ch2 expansion hub Servo
        tailDrive = hardwareMap.get(DcMotor.class, "tailDrive"); //ch3 expansion hub Motor

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        armDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        clawServo1.setDirection(Servo.Direction.FORWARD);
        clawServo2.setDirection(Servo.Direction.FORWARD);
        tailDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        armDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulderDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tailDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tailDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        armDrive1.setTargetPosition(0);
        armDrive2.setTargetPosition(0);
        shoulderDrive.setTargetPosition(0);
        tailDrive.setTargetPosition(0);

        waitForStart();
        runtime.reset();

        //Launch Threads
        Motion motion = new Motion(frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive, gamepad1);
        Arm arm = new Arm(armDrive1, armDrive2, gamepad2);
        Shoulder shoulder = new Shoulder(shoulderDrive, arm, gamepad2);
        Wrist wrist = new Wrist(wristServo, shoulder, gamepad2);
        Claw claw = new Claw(gamepad2, clawServo1, clawServo2, shoulder);
        Tail tail = new Tail(tailDrive, gamepad2);

        arm.setShoulder(shoulder);

        motion.start();
        arm.start();
        shoulder.start();
        wrist.start();
        claw.start();
        tail.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Stick", "x (%.2f), y (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Right Stick", "x (%.2f), y (%.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Arm Mode", armDrive1.getMode());
            telemetry.addData("Shoulder Ratio", "(%.2f)", shoulder.shoulderAngle());
            telemetry.addData("Arm Count1", "(%7d)", arm.getArmCounts1());
            telemetry.addData("Arm Count2", "(%7d)", arm.getArmCounts2());
            telemetry.addData("Wrist Count", "(%7f)", wrist.getWristCounts());
            telemetry.addData("Shoulder Count", "(%7d)", shoulder.getShoulderCounts());
            telemetry.addData("Claw 1 Count", "(%.2f)", claw.getClaw1Counts());
            telemetry.addData("Claw 2 Count", "(%.2f)", claw.getClaw2Counts());
            telemetry.addData("Tail Count", "(%7d)", tail.getTailCounts());
            telemetry.addData("Front Left Motor", "(%7d)", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Motor", "(%7d)", frontRightDrive.getCurrentPosition());
            telemetry.addData("Rear Left Motor", "(%7d)", rearLeftDrive.getCurrentPosition());
            telemetry.addData("Rear Right Motor", "(%7d)", rearRightDrive.getCurrentPosition());
            telemetry.update();
        }

        motion.interrupt();
        arm.interrupt();
        shoulder.interrupt();
        wrist.interrupt();
        claw.interrupt();
        tail.interrupt();
        try {
            motion.join();
            arm.join();
            shoulder.join();
            wrist.join();
            claw.join();
            tail.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}


