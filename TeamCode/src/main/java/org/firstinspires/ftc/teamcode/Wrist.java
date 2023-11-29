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

    private static double MAX_WRIST = 0.8;
    private static double MIN_WRIST = 0.35;
    private static double BOARD_ANGLE = 0.22; //60 degrees   originally 0.222

    private static double FLOOR_FRONT = 0.166; //44.7 degrees   originally 0.166
    private static double BOARD_FRONT = 0.32;
    private static double BOARD_BACK = 1- FLOOR_FRONT-BOARD_ANGLE;
    private static double FLOOR_BACK = 1-FLOOR_FRONT;

    private double delta = 0;
    private double pos;

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

            //Multiplies the shoulder's ratio and the range of wrist angles
            double shoulderAngle = shoulder.shoulderAngle();
            double wristAngle = (MAX_WRIST - MIN_WRIST) * shoulderAngle;
            double frontRatio = shoulderAngle/BOARD_FRONT;
            double frontBoardRatio = (shoulderAngle-FLOOR_FRONT)/BOARD_FRONT;

            if (shoulderAngle <= FLOOR_FRONT) {
                pos = MIN_WRIST * (1-frontRatio);
            } else if (shoulderAngle <= BOARD_FRONT){
                pos = 0.208 + BOARD_ANGLE * (1-frontBoardRatio);
            } else if (shoulderAngle <= BOARD_BACK){

            } else if (shoulderAngle <= FLOOR_BACK){

            }else {
                    pos = wristAngle + MIN_WRIST;

            }
            /*
            if (gamepad.back) {
                delta -= 0.01;
            } else if (gamepad.start) {
                delta += 0.01;

            }

             */
            wristServo.setPosition(pos);
        }
    }

    }