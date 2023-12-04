package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Shoulder extends Thread {
    //Variables for shoulder speed
    public static double SHOULDER_SPEED = 0.5;
    public static double MIN_SHOULDER_SPEED = -0.5;
    public static double MAX_SHOULDER_SPEED = 1;

    //Variables for pre-set positions
    public static int STRAIGHT_OUT = -395;
    public static int STRAIGHT_UP = -1370;
    public static int AGAINST_WALL = -1773;

    //Setting up variables for min and max pos
    private int MIN_POS;
    private int MAX_POS;

    //Pre-set min and max pos based on if the arm is in or out
    public static int MIN_POS_ARM_IN = 0;
    public static int MAX_POS_ARM_IN = -2657;
    public static int MIN_POS_ARM_OUT = -225;
    public static int MAX_POS_ARM_OUT = -2417;

    //Amount arm will move for manual adjustments
    public static int SHOULDER_MANUAL = 100;

    //Setting up vars of threading
    private DcMotor shoulderDrive;
    private Arm arm;
    private Gamepad gamepad;
    //Var for motor counts
    private int totalCounts;

    //Threading
    public  Shoulder(DcMotor shoulderDrive, Arm arm, Gamepad gamepad) {
        this.shoulderDrive = shoulderDrive;
        this.arm = arm;
        this.gamepad = gamepad;
    }

    /**
     * Function to get motor counts
     * @return total counts
     */
    protected int getShoulderCounts() {return totalCounts;}

    /**
     * Gets the ratio of shoulder base off its position
     * @return double between 0.0-1.0
     */
    public double getShoulderRatio() {
        //Sets a position variable of the robot's current position
        double pos = shoulderDrive.getCurrentPosition();
        //Checks to see if the arm is between 0 and -225
        if (pos > MIN_POS_ARM_OUT) {
            //Returns a double between 0.0-1.0 based on where the shoulder is between 0 and -225
            return Range.clip(((double) pos - MIN_POS_ARM_IN) / ((double) MIN_POS_ARM_OUT - (double)MIN_POS_ARM_IN), 0.0, 1.0);
        //Checks to see if the arm is between -2417 and -2657
        } else if (pos < MAX_POS_ARM_OUT) {
            //Returns a double between 0.0-1.0 based on where the shoulder is between -2417 and -2657
            return Range.clip(((double) pos - MAX_POS_ARM_IN) / ((double) MAX_POS_ARM_OUT - (double) MAX_POS_ARM_IN), 0.0, 1.0);
        } else {
            //Returns 1.0 if the arm is outside the prior two ranges
            return 1.0;
        }
    }

    public double shoulderAngle() {
        return (double) shoulderDrive.getCurrentPosition() / MAX_POS_ARM_IN;
    }

    /**
     * Sets the position of the shoulder
     * @param power double, power of the shoulder motor
     * @param position int, position/angle to set the shoulder to
     */
    private  void  setPosition(double power, int position) {
        //Sets the power to the inputted power, clips the power to make sure it is within 0-1
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);
        //Sets the position of the shoulder
        shoulderDrive.setTargetPosition(position);
        shoulderDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulderDrive.setPower(power);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            //Sets total counts to the shoulder's current position
            totalCounts = shoulderDrive.getCurrentPosition();

            //Sets the min pos to an int value based on how far the arm is out
            MIN_POS = (int) Math.round(arm.getArmRatio() * (MIN_POS_ARM_OUT-MIN_POS_ARM_IN) + MIN_POS_ARM_IN);
            //Sets the max pos to an int value based on how far the arm is out
            MAX_POS = (int) Math.round(arm.getArmRatio() * (MAX_POS_ARM_OUT-MAX_POS_ARM_IN) + MAX_POS_ARM_IN);


                if (gamepad.dpad_left) {
                    int pos = shoulderDrive.getCurrentPosition() + SHOULDER_MANUAL;
                    setPosition(SHOULDER_SPEED, Range.clip(pos, MAX_POS, MIN_POS));
                    //Lower manually
                } else if (gamepad.dpad_right) {
                    int pos = shoulderDrive.getCurrentPosition() -SHOULDER_MANUAL;
                    setPosition(SHOULDER_SPEED, Range.clip(pos, MAX_POS, MIN_POS));
                    //Raise manually
                    }


/*
            double SHOULDER_SPEED = gamepad.right_stick_y;
            int pos = (SHOULDER_SPEED >= 0) ? MIN_POS:MAX_POS;
            setPosition(SHOULDER_SPEED, pos);

 */

                if (gamepad.y) {
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
