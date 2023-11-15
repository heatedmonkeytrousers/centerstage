package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends Thread{
    /*
    Likely going to be a servo
    servoAngle when facing up = 180-theta
    servoAngle when facing board = 300-theta
    When motor is replaced, get counts for 180 degrees
    Extend Shoulder class to get the shoulder's angle
    Measure out clicks to angle ratio when robot is ready
     */

    private static double GROUND_POS = 0.1;
    private static double SAFE_POS = 0.9;
    private static double SCORING_POS = 0.6;
    private static double totalCounts;
    private Servo wristServo;
    private Gamepad gamepad;

    public Wrist(Servo wristServo, Gamepad gamepad) {
        this.wristServo = wristServo;
        this.gamepad = gamepad;
    }

    //Change these values after testing
    public void groundWrist () {wristServo.setPosition(GROUND_POS);}
    public void safeWrist () {wristServo.setPosition(SAFE_POS);}
    public void scoringWrist() {wristServo.setPosition(SCORING_POS);}

    protected double getWristCounts() {
        return totalCounts;
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            totalCounts = wristServo.getPosition();

            //Fine tuning for testing

            if (gamepad.back) {
                wristServo.setPosition(wristServo.getPosition() - 0.001);
            } else if (gamepad.start) {
                wristServo.setPosition(wristServo.getPosition() + 0.001);

        }
            /*
            else if (gamepad.start) {
                groundWrist();
            } else if (gamepad.back) {
                safeWrist();
            } else if (gamepad.left_stick_button) {
                scoringWrist();
            }

             */



        }
    }

    }


