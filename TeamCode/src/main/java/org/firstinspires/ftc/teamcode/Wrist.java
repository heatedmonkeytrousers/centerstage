package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    /*
    Likely going to be a servo
    servoAngle when facing up = 180-theta
    servoAngle when facing board = 300-theta
    When motor is replaced, get counts for 180 degrees
    Extend Shoulder class to get the shoulder's angle
    Measure out clicks to angle ratio when robot is ready
     */


    private Servo wrist;

    public Wrist(Servo wrist) {
        this.wrist = wrist;
    }




}
