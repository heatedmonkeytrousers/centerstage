package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Claw extends Thread {

    private final Servo linear2;
    private final Gamepad gamepad;

    public enum Direction {
        FORWARD,
        RIGHT,
        BACKWARD,
        LEFT
    }

    public Claw(Gamepad gamepad, Servo linear) {
        this.gamepad = gamepad;
        this.linear2 = linear;
    }

    double left = 0.58;
    double right = 0.3;

    @Override
    public void run() {

            if(gamepad.y) {
                // move to 0 degrees
                linear2.setPosition(0.2);
            }
            if (gamepad.x) {
                // extends to move claw
                linear2.setPosition(0.275);
            }

        }
    }