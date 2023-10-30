package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Thread{
    private static double ARM_SPEED = 1;
    private static double MIN_ARM_SPEED = -1;
    private static double MAX_ARM_SPEED = 1;
    private static int ARM_MANUAL = 100;

    private static int MIN_POS = 0;
    private static int MAX_POS = 1550;

    private DcMotor armDrive;
    private Shoulder shoulder;
    private int totalCounts;
    private Gamepad gamepad;



    public Arm(DcMotor armDrive, Gamepad gamepad) {
        this.armDrive = armDrive;
        this.gamepad = gamepad;
    }

    public void setShoulder(Shoulder shoulder) {
        this.shoulder = shoulder;
    }

    protected int getArmCounts() {
        return totalCounts;
    }

    /**
     * Gets the ratio from 0 to 1 inclusive of how far the arm is extended
     * @return [0, 1]
     */
    public double getArmRatio() {return Range.clip((double) totalCounts / (double) MAX_POS, 0.0, 1.0);}

    private void setPosition(double power, int position) {
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        armDrive.setTargetPosition(position);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setPower(power);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {

            if (shoulder != null) {
                MAX_POS = (int) Math.round(shoulder.getShoulderRatio());
            } else {
                MAX_POS = 0;
            }
            totalCounts = armDrive.getCurrentPosition();

            if (gamepad.dpad_left) {
                //Manual down
                int pos = armDrive.getCurrentPosition() - ARM_MANUAL;
                setPosition(ARM_SPEED, Range.clip(pos, MIN_POS, MAX_POS));

            } else if (gamepad.dpad_right) {
                //Manual up
                int pos = armDrive.getCurrentPosition() + ARM_MANUAL;

                setPosition(ARM_SPEED, Range.clip(pos, MIN_POS, MAX_POS));

            } else if (gamepad.dpad_up) {
                //Fully out
                setPosition(ARM_SPEED, MAX_POS);

            } else if (gamepad.dpad_down) {
                //Fully in
                setPosition(ARM_SPEED, MIN_POS);

            }



        }
    }
}
