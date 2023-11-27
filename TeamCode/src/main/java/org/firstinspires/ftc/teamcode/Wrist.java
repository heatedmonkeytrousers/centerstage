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
    private static double BOARD_ANGLE = 0.222; //60 degrees

    private static double FLOOR_FRONT = 0.166; //44.7 degrees
    private static double BOARD_FRONT = FLOOR_FRONT + BOARD_ANGLE;
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

            //Multiplies the shoulder's ratio and the range of wrist angles and sets the wrist's position to it
            double shoulderAngle = shoulder.shoulderAngle();
            double wristAngle = (MAX_WRIST - MIN_WRIST) * shoulderAngle;

            if (shoulderAngle <= FLOOR_FRONT) {
                pos = MIN_WRIST - wristAngle;
            } else if (shoulderAngle <= BOARD_FRONT){
                pos = MIN_WRIST - wristAngle + BOARD_ANGLE;
            } else if (shoulderAngle <= BOARD_BACK){

            } else if (shoulderAngle <= FLOOR_BACK){

            }else {
                    pos = wristAngle + MIN_WRIST;

            }
            wristServo.setPosition(pos);

            //Fine tuning for testing
            if (gamepad.back) {
                delta -= 0.01;
            } else if (gamepad.start) {
                delta += 0.01;

             }

        }
    }

    }