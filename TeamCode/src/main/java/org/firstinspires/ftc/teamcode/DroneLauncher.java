package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher extends Thread{
    private Servo servo;
    private Gamepad gamepad;

    public DroneLauncher(Gamepad gamepad,Servo servo) {
        this.gamepad = gamepad;
        this.servo = servo;
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            if (gamepad.start) {
                servo.setPosition(0);
            } else if (gamepad.back) {
                servo.setPosition(0.0925);
            }
        }
    }
}
