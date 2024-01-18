package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Shoulder extends Thread {
    //Variables for shoulder speed
    public static double MIN_SHOULDER_SPEED = -0.5;
    public static double MAX_SHOULDER_SPEED = 1;

    //Setting up variables for min and max pos
    private int MIN_POS;
    private int MAX_POS;

    //Pre-set min and max pos based on if the arm is in or out
    public static int MIN_POS_ARM_IN = -20;
    public static int MAX_POS_ARM_IN = -2657;
    public static int MIN_POS_ARM_OUT = -225;
    public static int MAX_POS_ARM_OUT = -2417;

    //Amount arm will move for manual adjustments
    public static int SHOULDER_MANUAL = 100;

    //Setting up vars of threading
    private final DcMotor shoulderDrive;
    private final Arm arm;
    private final Gamepad gamepad;
    //Var for motor counts
    private int totalCounts;

    /**
     * Constructor for the shoulder
     * @param shoulderDrive the motor for the shoulder
     * @param arm the motor for the arm
     * @param gamepad the gamepad used for controlling the shoulder
     */
    public  Shoulder(DcMotor shoulderDrive, Arm arm, Gamepad gamepad) {
        this.shoulderDrive = shoulderDrive;
        this.arm = arm;
        this.gamepad = gamepad;
        this.shoulderDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    /**
     * Gets the ratio of the shoulder's current position to the shoulder's total range
     * @return double between 0.0-1.0
     */
    public double shoulderAngle() {
        return (double) shoulderDrive.getCurrentPosition() / MAX_POS_ARM_IN;
    }

    public boolean isUp() {
        return shoulderDrive.getCurrentPosition() < -300;
    }

    /**
     * Sets the position of the shoulder
     * @param power double, power of the shoulder motor
     * @param position int, position/angle to set the shoulder to
     */
    public  void  setShoulderPosition(double power, int position) {
        //Sets the power to the inputted power, clips the power to make sure it is within 0-1
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);
        //Sets the position of the shoulder
        shoulderDrive.setTargetPosition(position);
        shoulderDrive.setPower(power);
    }

    /**
     * Sets the position of the shoulder based on a value 0.0-1.0
     * @param power the power of the shoulder
     * @param position a value 0.0-1.0 that sets the position of the shoulder
     */
    public  void setShoulderPosition(double power, double position) {
        //Sets the power to the inputted power, clips the power to make sure it is within 0-1
        power = Range.clip(power, MIN_SHOULDER_SPEED, MAX_SHOULDER_SPEED);
        //Sets the position of the shoulder based on the position given
        //Multiplies the range of the shoulder movement by the position and adds the min pos
        shoulderDrive.setTargetPosition((int) ((MAX_POS_ARM_OUT-MIN_POS_ARM_IN) * position) + MIN_POS_ARM_IN);
        shoulderDrive.setPower(power);
    }

    @Override
    public void run() {
        //Initializes position, power multiplier, and hold var
        int pos;
        double PF;
        boolean hold = false;
        while (!isInterrupted()) {
            //Sets total counts to the shoulder's current position
            totalCounts = shoulderDrive.getCurrentPosition();

            //Sets the min pos to an int value based on how far the arm is out
            MIN_POS = (int) Math.round(arm.getArmRatio() * (MIN_POS_ARM_OUT-MIN_POS_ARM_IN) + MIN_POS_ARM_IN);
            //Sets the max pos to an int value based on how far the arm is out
            MAX_POS = (int) Math.round(arm.getArmRatio() * (MAX_POS_ARM_OUT-MAX_POS_ARM_IN) + MAX_POS_ARM_IN);

            //Sets the shoulder speed to a value -1 through 1 based on the right stick
            if (gamepad != null) {
                double SHOULDER_SPEED = gamepad.right_stick_y;
                double power;
                //If the shoulder is not told to hold it's position and the speed is less than 0.15
                //Make the shoulder hold it's current position
                if (!hold && Math.abs(SHOULDER_SPEED) < 0.15) {
                    //Set the pos to the shoulder's current position
                    pos = totalCounts;
                    power = 0.75;
                    setShoulderPosition(power, pos);
                    hold = true;
                    //If the shoulder speed is greater than 0.15
                } else if (SHOULDER_SPEED > 0.15) {
                    //Move the shoulder towards the min pos
                    pos = MIN_POS;
                    //If the shoulder is up high enough, lower the speed
                    PF = isUp() ? 0.5 : 0.75;
                    //Multiplies speed by the power factor
                    power = SHOULDER_SPEED * PF;
                    setShoulderPosition(power, pos);
                    hold = false;
                    //If the shoulder speed is less than -0.15
                } else if (SHOULDER_SPEED < -0.15) {
                    //Moves the shoulder towards the max pos
                    pos = MAX_POS;
                    //If the shoulder is up high enough, lower the speed
                    PF = isUp() ? 0.5 : 0.75;
                    //Multiplies speed by the power factor
                    power = Math.abs(SHOULDER_SPEED) * PF;
                    setShoulderPosition(power, pos);
                    hold = false;
                }
                //Pre set buttons for setting the position
                if (gamepad.a) {
                    //Driving position
                    setShoulderPosition(0.75, 0);
                } else if (gamepad.b) {
                    //Between line 2 and 3 front
                    setShoulderPosition(0.75, -655); //-739
                } else if (gamepad.y) {
                    //Between line 1 and 2 front
                    setShoulderPosition(0.75, -525); //-655
                } else if (gamepad.x) {
                    //Driving position
                    setShoulderPosition(0.5, -60);
                } else if (gamepad.back) {
                    MIN_POS_ARM_IN += 5;
                    setShoulderPosition(0.5, MIN_POS_ARM_IN);
                } else if (gamepad.dpad_left) {
                    //Between line 1 and 2 back
                    setShoulderPosition(0.75, -1800); // -1936
                } else if (gamepad.dpad_right) {
                    //Between line 2 and 3 back
                    setShoulderPosition(0.75, -739); // -1800
                } else if (gamepad.dpad_up) {
                    //Straight up and down
                    setShoulderPosition(0.75, -1325); // -1325
                }
            }
        }
    }
}
