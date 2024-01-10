package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This class is a super class for all autonomous children
 */

public class AutonomousOpMode extends StandardSetupOpMode {

    public enum COLOR {
        RED,
        BLUE
    }

    public enum HAMSTER_POS {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum START_POS {
        FAR,
        NEAR
    }

    private static final double SMALL_TURN = 25;
    private static final double LARGE_TURN = 38;

    protected SampleMecanumDrive drive;

    //poses
    protected Pose2d startPose = new Pose2d();
    protected Pose2d dropPose;
    protected START_POS startPos;
    protected HAMSTER_POS hamsterPos;
    protected COLOR color;

    //If red alliance then yScale is -1.0
    //If blue alliance then yScale is 1.0
    protected double yScale;
    protected double sScale;

    protected void setup(HardwareMap hardwareMap, START_POS startPos, HAMSTER_POS hamsterPos, COLOR color) {
        // Variable setup
        drive = new SampleMecanumDrive(hardwareMap);
        this.startPos = startPos;
        this.hamsterPos = hamsterPos;
        this.yScale = (color == COLOR.RED) ? -1.0 : 1.0;
        this.sScale = (startPos == START_POS.FAR) ? 1.0 : -1.0;

        // Reset the 30 second runtime timer
        runtime.reset();

        telemetry.addData("hamsterPos", hamsterPos);
        telemetry.addData("color", color);
        telemetry.update();

        // Drop angle
        double dropAngle = 0;
        switch (hamsterPos) {
            case LEFT:
                if (color == COLOR.RED)
                    dropAngle = ((startPos == START_POS.FAR) ? SMALL_TURN : LARGE_TURN);
                else
                    dropAngle = ((startPos == START_POS.FAR) ? LARGE_TURN : SMALL_TURN);
                break;
            case RIGHT:
                if (color == COLOR.RED)
                    dropAngle = ((startPos == START_POS.FAR) ? -LARGE_TURN : -SMALL_TURN);
                else
                    dropAngle = ((startPos == START_POS.FAR) ? -SMALL_TURN : -LARGE_TURN);
                break;
            case CENTER:
                dropAngle = 0;
                break;
        }

        // Common drop pose for all starting positions
        dropPose = new Pose2d(16.5, -2.5 * sScale * yScale, Math.toRadians(dropAngle));

        // Class Setup
        super.arm.setShoulder(shoulder);
        super.wrist.start();
        super.claw.rightClose();
        super.claw.leftClose();
    }
}
