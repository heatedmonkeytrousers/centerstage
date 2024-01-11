package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Thread{
    //Min and max speed of the arm
    private static final double MIN_ARM_SPEED = -1;
    private static final double MAX_ARM_SPEED = 1;

    //Min and max pos of the arm
    private static final int MIN_POS = 0;
    private static final int MAX_POS = 2180;

    //Vars for the arm motors
    private final DcMotor armDrive1;
    private final DcMotor armDrive2;
    //Var for the shoulder and gamepad for the constructor
    private Shoulder shoulder;
    private final Gamepad gamepad;
    //Vars for keeping track of the arm's position
    private int totalCounts1;
    private int totalCounts2;

    /**
     * Constructor for the arm
     * @param armDrive1 the motor for the first arm
     * @param armDrive2 the motor for the second arm
     * @param gamepad the gamepad used to control the arm
     */
    public Arm(DcMotor armDrive1, DcMotor armDrive2, Gamepad gamepad, Shoulder shoulder) {
        this.armDrive1 = armDrive1;
        this.armDrive2 = armDrive2;
        this.gamepad = gamepad;
        this.shoulder = shoulder;
        this.armDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.armDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets the value of the shoulder for initialization
     * @param shoulder the shoulder motor used
     */
    public void setShoulder(Shoulder shoulder) {
        this.shoulder = shoulder;
    }

    /**
     * Gets the position of the first arm motor
     * @return totalCounts1, the position of the first arm motor
     */
    protected int getArmCounts1() {
        return totalCounts1;
    }

    /**
     * Gets the position of the second arm motor
     * @return totalCounts2, the position of the second arm motor
     */
    protected int getArmCounts2() {
        return totalCounts2;
    }

    /**
     * Gets the ratio from 0 to 1 inclusive of how far the arm is extended
     * @return [0, 1]
     */
    public double getArmRatio() {return Range.clip((double) totalCounts1 / (double) MAX_POS, 0.0, 1.0);}

    /**
     * Sets the position of the arm
     * @param power the power of the arm
     * @param position a value 0-2180 that sets the arm position
     */
    public void setArmPosition(double power, int position) {
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        armDrive1.setPower(power);
        armDrive2.setPower(power);
        armDrive2.setTargetPosition(position);
        armDrive1.setTargetPosition(position);
    }

    /**
     * Sets the position of the arm based on a value 0.0-1.0
     * @param power the power of the arm
     * @param position a value 0.0-1.0 which determines the position
     */
    public void setArmPosition(double power, double position) {
        //Makes sure the power is between 0-1
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        //Sets the power of both arm motors
        armDrive1.setPower(power);
        armDrive2.setPower(power);
        //Sets the position of both arms based on the position given
        //Multiplies the range of the arm movement by the position and adds the min pos (in case min pos isn't zero)
        armDrive2.setTargetPosition((int) ((MAX_POS-MIN_POS) * position) + MIN_POS);
        armDrive1.setTargetPosition((int) ((MAX_POS-MIN_POS) * position) + MIN_POS);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {

            //Gets the current position of the arm
            totalCounts1 = armDrive1.getCurrentPosition();
            totalCounts2 = armDrive1.getCurrentPosition();

            //Sets the arm speed to a number -1 through 1 based on the left stick's position
            double ARM_SPEED = gamepad.left_stick_y;
            //If the arm speed is positive, then we move the arm towards the min pos
            //If the arm speed is negative, then we move the arm towards the max pos
            int pos = (ARM_SPEED >= 0) ? MIN_POS:MAX_POS;
            //Sets the arm pos based on the pos we just calculated
            setArmPosition(ARM_SPEED, pos);

            if (shoulder != null) {
                int shoulderCounts = shoulder.getShoulderCounts();
                if (shoulderCounts > Shoulder.MIN_POS_ARM_OUT) {
                    double shoulderRatio = (double) (shoulderCounts - Shoulder.MIN_POS_ARM_IN) / (double) (Shoulder.MIN_POS_ARM_OUT-Shoulder.MIN_POS_ARM_IN);
                    double armRatio = getArmRatio();
                    if (armRatio > shoulderRatio) {
                        int newPos = (int) ((double) (Shoulder.MIN_POS_ARM_OUT-Shoulder.MIN_POS_ARM_IN) * armRatio) + Shoulder.MIN_POS_ARM_IN;
                        shoulder.setShoulderPosition(0.75, newPos);
                    }
                }
            }
        }
    }
}
