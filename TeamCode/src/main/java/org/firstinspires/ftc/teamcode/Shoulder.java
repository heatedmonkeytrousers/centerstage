package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Shoulder extends Thread {
    public static double SHOULDER_SPEED = 1;
    public static double MIN_SHOULDER_SPEED = -1;
    public static double MAX_SHOULDER_SPEED = 1;

    public static int STRAIGHT_OUT = -395;
    public static int STRAIGHT_UP = -1370;
    public static int AGAINST_WALL = -1773;

    private int MIN_POS;
    private int MAX_POS;

    public static int MIN_POS_ARM_IN = 0;
    public static int MAX_POS_ARM_IN = -2657;
    public static int MIN_POS_ARM_OUT = -225;
    public static int MAX_POS_ARM_OUT = -2417;

    public static int SHOULDER_MANUAL = 100;

    private DcMotor shoulderDrive;
    private Arm arm;
    private int totalCounts;
    private Gamepad gamepad;



    public  Shoulder(DcMotor shoulderDrive, Arm arm, Gamepad gamepad) {
        this.shoulderDrive = shoulderDrive;
        this.arm = arm;
        this.gamepad = gamepad;
    }

    protected int getShoulderCounts() {return totalCounts;}

    public double getShoulderRatio() {
        double pos = shoulderDrive.getCurrentPosition();
        if (pos > MIN_POS_ARM_OUT) {
            return Range.clip(((double) pos - MIN_POS_ARM_IN) / ((double) MIN_POS_ARM_OUT - (double)MIN_POS_ARM_IN), 0.0, 1.0);
        } else if (pos < MAX_POS_ARM_OUT) {
            return Range.clip(((double) pos - MAX_POS_ARM_IN) / ((double) MAX_POS_ARM_OUT - (double) MAX_POS_ARM_IN), 0.0, 1.0);
        } else {
            return 1.0;
        }
    }

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

            MIN_POS = (int) Math.round(arm.getArmRatio() * (MIN_POS_ARM_OUT-MIN_POS_ARM_IN) + MIN_POS_ARM_IN);
            MAX_POS = (int) Math.round(arm.getArmRatio() * (MAX_POS_ARM_OUT-MAX_POS_ARM_IN) + MAX_POS_ARM_IN);

                if (gamepad.left_bumper) {
                    int pos = shoulderDrive.getCurrentPosition() + SHOULDER_MANUAL;
                    setPosition(SHOULDER_SPEED, Range.clip(pos, MAX_POS, MIN_POS));
                    //Lower manually
                } else if (gamepad.right_bumper) {
                    int pos = shoulderDrive.getCurrentPosition() -SHOULDER_MANUAL;
                    setPosition(SHOULDER_SPEED, Range.clip(pos, MAX_POS, MIN_POS));
                    //Raise manually
                } else if (gamepad.y) {
                    setPosition(SHOULDER_SPEED, STRAIGHT_UP);
                    //180 degree angle
                } else if (gamepad.x) {
                    setPosition(SHOULDER_SPEED, AGAINST_WALL);
                    //Close against the board
                } else if (gamepad.b) {
                    setPosition(SHOULDER_SPEED, STRAIGHT_OUT);
                    //90 degree angle
                } else if (gamepad.a) {
                    setPosition(SHOULDER_SPEED, MIN_POS);

                    //Reset position

            }
        }
    }
}
