package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Arm extends Thread{
    private static double ARM_SPEED = 1;
    private static double MIN_ARM_SPEED = -1;
    private static double MAX_ARM_SPEED = 1;

    private DcMotor armDrive;
    private int totalCounts;
    private Gamepad gamepad;

    public Arm(DcMotor armDrive, Gamepad gamepad) {
        this.armDrive = armDrive;
        this.gamepad = gamepad;
    }

    public int getArmCounts() {
        return totalCounts;
    }

    private void setPosition(double power, int position) {
        power = Range.clip(power, MIN_ARM_SPEED, MAX_ARM_SPEED);
        armDrive.setTargetPosition(position);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setPower(power);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            totalCounts = armDrive.getCurrentPosition();

            if (gamepad.a) {
                setPosition(ARM_SPEED, 100);
            } else if (gamepad.b) {
                setPosition(ARM_SPEED, 0);
            }
        }
    }
}
