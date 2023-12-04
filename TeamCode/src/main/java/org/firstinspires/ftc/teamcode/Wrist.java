package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;
import java.util.ArrayList;

public class Wrist extends Thread{

    //Setting up vars for telemetry
    private static double totalCounts;
    //Setting up vars for threading
    private Servo wristServo;
    private Shoulder shoulder;
    private Gamepad gamepad;


    private static ArrayList<Double> s = new ArrayList<Double>(List.of(
            0.0, //On the floor front
            0.16, //90 degree front
            0.16, //90 degree front
            0.30, //60 degree front
            0.64, //60 degree back
            0.83, //90 degree back
            0.83, //90 degree back
            0.96)); //On the floor back
    private static ArrayList<Double> w = new ArrayList<Double>(List.of(
            0.35, //Wrist on floor front
            0.174, //Wrist flat 90 degree front
            0.43, //Wrist at 60 90 degree front
            0.31, //Wrist at 60 front
            0.86, //Wrist at 60 back
            0.68, //Wrist at 60 90 degree back
            0.88, //Wrist flat 90 degree back
            0.8 //Wrist on floor back
    ));

    //Threading the wrist
    public Wrist(Servo wristServo, Shoulder shoulder, Gamepad gamepad) {
        this.wristServo = wristServo;
        this.shoulder = shoulder;
        this.gamepad = gamepad;
    }

    protected double getWristCounts() {
        return totalCounts;
    }

    //Figures out what range we're in and sets the wrist's position accordingly
    public double wristPos(double shoulderAngle) {
        int i = 0;
        //Goes through the shoulder list
        for (;i < s.size()-1; i++) {
            //Finds if the shoulder angle is less than the next item in the list
            if (shoulderAngle < s.get(i + 1)) {
                //If it is, then we have the range our shoulder is in
                break;
            }
        }
        //Using the i value we found from the for loop, we find the position of the wrist
        //We first the ratio of the shoulder angle to the range it is in
        //Then we multiply it by the range of the wrist angles for that range
        //Lastly we add the wrist pos found at the index
        return ( (shoulderAngle - s.get(i)) / (s.get(i+1) - s.get(i)) ) * (w.get(i+1) - w.get(i)) + w.get(i);
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            //Gets the total counts for telemetry purposes
            totalCounts = wristServo.getPosition();

            //Set the position of the wrist based on the shoulder's location
            wristServo.setPosition(wristPos(shoulder.shoulderAngle()));
        }
    }

    }