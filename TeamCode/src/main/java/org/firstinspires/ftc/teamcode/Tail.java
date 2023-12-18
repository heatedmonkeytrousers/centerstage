package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Tail extends Thread{
    private static double TAIL_SPEED = 0.5;
    private static double MIN_TAIL_SPEED = -1;
    private static double MAX_TAIL_SPEED = 1;
    private static int MIN_POS = 0;
    private static int MAX_POS = 929;
    private int tailCounts;
    private boolean inAir = false;

    private DcMotor tail;
    private Gamepad gamepad;

    public Tail(DcMotor tail, Gamepad gamepad) {
        this.tail = tail;
        this.gamepad = gamepad;
        this.tail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected int getTailCounts() {
        return tailCounts;
    }

    private void setPosition(double power, int position) {
        power = Range.clip(power, MIN_TAIL_SPEED, MAX_TAIL_SPEED);
        tail.setPower(power);
        tail.setTargetPosition(position);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {

            tailCounts = tail.getCurrentPosition();

            if (gamepad.start) {
                if (inAir) {
                    setPosition(TAIL_SPEED, MIN_POS);
                    inAir = false;
                } else {
                    setPosition(TAIL_SPEED, MAX_POS);
                    inAir = true;
                }


            }

            /*
            if (gamepad.dpad_up) {
                int pos = tail.getCurrentPosition() + 100;
                setPosition(TAIL_SPEED, pos);
            } else if (gamepad.dpad_down) {
                int pos = tail.getCurrentPosition() - 100;
                setPosition(TAIL_SPEED, pos);
            }

             */


        }
    }
}

