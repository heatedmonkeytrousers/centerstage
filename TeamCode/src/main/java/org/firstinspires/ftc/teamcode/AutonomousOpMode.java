package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This class is a super class for all autonomous children
 */

public class AutonomousOpMode extends StandardSetupOpMode {

    protected SampleMecanumDrive drive;

    //poses
    protected Pose2d startPose = new Pose2d();
    protected Pose2d dropPose = new Pose2d(16.5, 0, 0);
    protected int position;

    protected void setup (HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
        this.position = 0;
    }
}
