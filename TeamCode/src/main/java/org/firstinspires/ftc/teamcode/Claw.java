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

    @Override
    public void run() {
        while (!isInterrupted()) {
            totalCountsOne = clawServo1.getPosition();
            totalCountsTwo = clawServo2.getPosition();

                if (gamepad.left_bumper) {
                    clawServo1.setPosition(OPEN);
                } else if (gamepad.left_trigger > 0) {
                    clawServo1.setPosition(CLOSE);
                }

                if (gamepad.right_bumper) {
                    clawServo2.setPosition(OPEN);
                } else if (gamepad.right_trigger > 0) {
                    clawServo2.setPosition(CLOSE);
                }

        }
    }
    }