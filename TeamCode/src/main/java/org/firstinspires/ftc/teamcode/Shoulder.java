package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Shoulder extends Thread {
    public static double SHOULDER_SPEED = 1;
    public static double MIN_SHOULDER_SPEED = -1;
    public static double MAX_SHOULDER_SPEED = 1;

    private DcMotor shoulderDrive;
    private int totalCounts;
    private Gamepad gamepad;

    public  Shoulder(DcMotor shoulderDrive, Gamepad gamepad) {
        this.shoulderDrive = shoulderDrive;
        this.gamepad = gamepad;
    }

    public int getShoulderCounts() {return totalCounts;}

    private  void  setPosition(double power, int position) {
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);
        shoulderDrive.setTargetPosition(position);
        shoulderDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderDrive.setPower(power);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            totalCounts = shoulderDrive.getCurrentPosition();
                if (gamepad.left_bumper) {
                    setPosition(SHOULDER_SPEED, shoulderDrive.getCurrentPosition() + 100);
                    //Lower manually
                } else if (gamepad.right_bumper) {
                    setPosition(SHOULDER_SPEED, shoulderDrive.getCurrentPosition() - 100);
                    //Raise manually
                } else if (gamepad.y) {
                    setPosition(SHOULDER_SPEED, -3388);
                    //90 degree angle
                } else if (gamepad.x) {
                    setPosition(SHOULDER_SPEED, -4680);
                    //Close against the board
                } else if (gamepad.b) {
                    setPosition(SHOULDER_SPEED, -5630);
                    //far against the board
                } else if (gamepad.a) {
                    setPosition(SHOULDER_SPEED, 0);
                    //Reset position
            }
        }
    }
}
