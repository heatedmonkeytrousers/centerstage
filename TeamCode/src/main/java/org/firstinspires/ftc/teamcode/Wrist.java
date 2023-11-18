package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Wrist extends Thread{
    /*
    Likely going to be a servo
    servoAngle when facing up = 180-theta
    servoAngle when facing board = 300-theta
    When motor is replaced, get counts for 180 degrees
    Extend Shoulder class to get the shoulder's angle
    Measure out clicks to angle ratio when robot is ready
     */

    private static double totalCounts;
    private Servo wristServo;
    private Shoulder shoulder;
    private Gamepad gamepad;

    private static double MAX_ARM_IN = 0.8;
    private static double MIN_ARM_IN = 0.35;

    public Wrist(Servo wristServo, Shoulder shoulder, Gamepad gamepad) {
        this.wristServo = wristServo;
        this.shoulder = shoulder;
        this.gamepad = gamepad;
    }

    protected double getWristCounts() {
        return totalCounts;
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            totalCounts = wristServo.getPosition();

            //Multiplies the shoulder's ratio and the range of wrist angles and sets the wrist's position to it
            wristServo.setPosition(shoulder.shoulderAngle() * (MAX_ARM_IN - MIN_ARM_IN) + MIN_ARM_IN);

            //Fine tuning for testing
            if (gamepad.back) {
                wristServo.setPosition(Range.clip(wristServo.getPosition() - 0.001, MIN_ARM_IN, MAX_ARM_IN));
            } else if (gamepad.start) {
                wristServo.setPosition(Range.clip(wristServo.getPosition() + 0.001, MIN_ARM_IN, MAX_ARM_IN));

        }

        }
    }

    }