package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Claw extends Thread {

    private final Servo clawServo1;
    private final Servo clawServo2;
    private final Gamepad gamepad;
    private final Shoulder shoulder;
    private double totalCountsOne;
    private double totalCountsTwo;

    private static double OPEN = 0.57;
    private static double CLOSE = 0.2;

    public static int STRAIGHT_UP = -1370;

    public Claw(Gamepad gamepad, Servo clawServo1, Servo clawServo2, Shoulder shoulder) {
        this.gamepad = gamepad;
        this.clawServo1 = clawServo1;
        this.clawServo2 = clawServo2;
        this.shoulder = shoulder;
    }

    protected double getClaw1Counts() {
        return totalCountsOne;
    }
    protected double getClaw2Counts() {
        return totalCountsTwo;
    }

    public void leftOpen() {
        clawServo1.setPosition(OPEN);
    }

    public void leftClose() {
        clawServo1.setPosition(CLOSE);
    }

    public void rightOpen() {
        clawServo2.setPosition(OPEN);
    }

    public void rightClose() {
        clawServo2.setPosition(CLOSE);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            totalCountsOne = clawServo1.getPosition();
            totalCountsTwo = clawServo2.getPosition();

                if (gamepad.left_bumper) {
                    leftOpen();
                } else if (gamepad.left_trigger > 0) {
                    leftClose();
                }

                if (gamepad.right_bumper) {
                    rightOpen();
                } else if (gamepad.right_trigger > 0) {
                    rightClose();
                }

        }
    }
    }