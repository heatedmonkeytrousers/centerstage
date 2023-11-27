package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Thread{
    private static double MIN_ARM_SPEED = -1;
    private static double MAX_ARM_SPEED = 1;
    private static int ARM_MANUAL = 100;

    private static int MIN_POS = 0;
    private static int MAX_POS = 2180;   //Originally 1550

    private static int SHOULDER_MAX;

    private DcMotor armDrive1;
    private DcMotor armDrive2;
    private Shoulder shoulder;
    private int totalCounts1;
    private int totalCounts2;
    private Gamepad gamepad;

    public Arm(DcMotor armDrive1, DcMotor armDrive2, Gamepad gamepad) {
        this.armDrive1 = armDrive1;
        this.armDrive2 = armDrive2;
        this.gamepad = gamepad;
        this.armDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.armDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setShoulder(Shoulder shoulder) {
        this.shoulder = shoulder;
    }

    protected int getArmCounts1() {
        return totalCounts1;
    }

    protected int getArmCounts2() {
        return totalCounts2;
    }

    /**
     * Gets the ratio from 0 to 1 inclusive of how far the arm is extended
     * @return [0, 1]
     */
    public double getArmRatio() {return Range.clip((double) totalCounts1 / (double) MAX_POS, 0.0, 1.0);}

    private void setPosition(double power, int position) {
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        armDrive1.setPower(power);
        armDrive2.setPower(power);
        armDrive2.setTargetPosition(position);
        armDrive1.setTargetPosition(position);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {

            if (shoulder != null) {
                SHOULDER_MAX = (int) Math.round(shoulder.getShoulderRatio());
            } else {
                SHOULDER_MAX = 0;
            }
            totalCounts1 = armDrive1.getCurrentPosition();
            totalCounts2 = armDrive1.getCurrentPosition();

            double ARM_SPEED = gamepad.left_stick_y;
            int pos = (ARM_SPEED >= 0) ? MIN_POS:MAX_POS;
            setPosition(ARM_SPEED, pos);

        }
    }
}
