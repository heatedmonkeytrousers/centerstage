package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.List;
import java.util.ArrayList;

public class Wrist extends Thread{

    //Setting up vars for telemetry
    private double totalCounts;
    //Setting up vars for threading
    private Servo wristServo;
    private AnalogInput wristAnalog;
    private Shoulder shoulder;
    private Gamepad gamepad;

    double position = 1.0;
    double delta = 0.01;

    private static ArrayList<Double> s = new ArrayList<Double>(List.of(
            0.00, //On the floor front
            0.16, //90 degree front
            0.16, //90 degree front
            0.30, //60 degree front
            0.64, //60 degree back
            0.83, //90 degree back
            0.83, //90 degree back
            0.96)); //On the floor back
    private static ArrayList<Double> w = new ArrayList<Double>(List.of(
            0.93, //Wrist on floor front
            1.0, //Wrist flat 90 degree front
            0.814, //Wrist at 60 90 degree front
            0.933, //Wrist at 60 front
            0.465, //Wrist at 60 back
            0.596, //Wrist at 60 90 degree back
            0.424, //Wrist flat 90 degree back
            0.525 //Wrist on floor back
    ));



    //Threading the wrist
    public Wrist(Servo wristServo, AnalogInput wristAnalog, Shoulder shoulder, Gamepad gamepad) {
        this.wristServo = wristServo;
        this.wristAnalog = wristAnalog;
        this.shoulder = shoulder;
        this.gamepad = gamepad;
    }

    protected double getWristAngleDegrees() {
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
/*
            if (gamepad.back) {
                //position = Range.clip(position - delta, 0.4, 1.0);
                totalCounts -= delta;
                wristServo.setPosition(totalCounts);
            } else if (gamepad.start) {
                //position = Range.clip(position + delta, 0.4, 1.0);
                totalCounts += delta;
                wristServo.setPosition(totalCounts);
            }
            
 */


            //Gets the total counts for telemetry purposes
            //totalCounts = wristServo.getPosition();

            //Set the position of the wrist based on the shoulder's location
            wristServo.setPosition(wristPos(shoulder.shoulderAngle()));


        }
    }

    }