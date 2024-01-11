package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Claw extends Thread {

    //Vars for both claw servos
    private final Servo clawServoLeft;
    private final Servo clawServoRight;
    //Vars for the gamepad and shoulder
    private final Gamepad gamepad;
    private final Shoulder shoulder;
    //Vars for the positions of each claw servo
    private double totalCountsLeft;
    private double totalCountsRight;

    //Value of the claw servo when it is open
    private static final double OPEN = 0.57;
    //Value of the claw servo when it is closed
    private static final double CLOSE = 0.2;

    /**
     * Constructor for the claw
     * @param gamepad the gamepad used to control the claw
     * @param clawServo1 the servo for the left claw
     * @param clawServo2 the servo for the right claw
     * @param shoulder the shoulder motor
     */
    public Claw(Gamepad gamepad, Servo clawServo1, Servo clawServo2, Shoulder shoulder) {
        this.gamepad = gamepad;
        this.clawServoLeft = clawServo1;
        this.clawServoRight = clawServo2;
        this.shoulder = shoulder;
    }

    /**
     * Gets the position of the left claw
     * @return totalCountsOne the position of the left claw
     */
    protected double getClawRightCounts() {
        return totalCountsRight;
    }
    /**
     * Gets the position of the left claw
     * @return totalCountsTwo the position of the left claw
     */
    protected double getClawLeftCounts() {
        return totalCountsLeft;
    }

    /**
     * Opens the left claw
     */
    public void leftOpen() {
        clawServoLeft.setPosition(OPEN);
    }

    /**
     * Closes the left claw
     */
    public void leftClose() {
        clawServoLeft.setPosition(CLOSE);
    }

    /**
     * Opens the rightClaw
     */
    public void rightOpen() {
        clawServoRight.setPosition(OPEN);
    }

    /**
     * Closes the right claw
     */
    public void rightClose() {
        clawServoRight.setPosition(CLOSE);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            //Gets the positions of the claws
            totalCountsLeft = clawServoLeft.getPosition();
            totalCountsRight = clawServoRight.getPosition();

            //If the left bumper is pressed
            if (gamepad.left_bumper) {
                //Closes the left claw
                leftClose();
            //If the left trigger is pressed
            } else if (gamepad.left_trigger > 0) {
                //Opens the left claw
                leftOpen();
            }

            //If the right bumper is pressed
            if (gamepad.right_bumper) {
                //Closes the right claw
                rightClose();
            //If the right trigger is pressed
            } else if (gamepad.right_trigger > 0) {
                //Opens the right claw
                rightOpen();
            }

        }
    }
    }