package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    //Likely going to be a servo
    //servoAngle when facing up = 180-theta
    //servoAngle when facing board = 300-theta

    public static double WRIST_SPEED = 1;
    public static double MIN_WRIST_SPEED = -1;
    public static double MAX_WRIST_SPEED = 1;

    private Servo wrist;

    public Wrist(Servo wrist) {
        this.wrist = wrist;
    }

    private void levelWrist(double theta) {
        double servoAngle = 180 - theta;
    }
}
