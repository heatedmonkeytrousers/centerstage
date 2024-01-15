package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

    private static final double FORWARD = 16.5;
    private static final double WIGGLE = 2.5;
    private static final double SMALL_TURN = 27;
    private static final double LARGE_TURN = 42;

    protected static final int ARM_DROP = 1600;
    protected static final int INITIAL_SHOULDER_RAISE = -75;
    protected int INITIAL_ARM_EXTENTION = 0; //How far the arm extends to drop the pixel on the tape

    protected SampleMecanumDrive drive;

    //poses
    protected Pose2d startPose = new Pose2d();
    protected Pose2d dropPose;
    protected START_POS startPos;

    // Trajectories
    protected Trajectory start;

    // Scale applied to flip the y-axis
    protected double yScale;

    protected void setup(HardwareMap hardwareMap, START_POS startPos, HAMSTER_POS hamsterPos, COLOR color) {
        // Variable setup
        drive = new SampleMecanumDrive(hardwareMap);
        this.startPos = startPos;
        this.hamsterPos = hamsterPos;

        //If red alliance then yScale is -1.0
        //If blue alliance then yScale is 1.0
        this.yScale = (color == COLOR.RED) ? -1.0 : 1.0;

        // Reset the 30 second runtime timer
        runtime.reset();

        // Telemetry
        telemetry.addData("hamsterPos", hamsterPos);
        telemetry.addData("color", color);
        telemetry.update();

        // Drop angle
        double deltaY; // Wiggle amount from the center
        double dropAngle; // Angle to turn for drop
        switch (hamsterPos) {
            case LEFT:
                if (color == COLOR.RED) {
                    dropAngle = ((startPos == START_POS.FAR) ? SMALL_TURN : 45);
                    deltaY = ((startPos == START_POS.FAR) ? WIGGLE : -WIGGLE);
                    INITIAL_ARM_EXTENTION = ((startPos == START_POS.FAR) ? 398:1262);
                }
                else {
                    dropAngle = ((startPos == START_POS.FAR) ? LARGE_TURN : SMALL_TURN);
                    deltaY = ((startPos == START_POS.FAR) ? -WIGGLE : WIGGLE);
                    INITIAL_ARM_EXTENTION = ((startPos == START_POS.FAR) ? 1262:398);
                }
                break;
            case RIGHT:
                if (color == COLOR.RED) {
                    dropAngle = ((startPos == START_POS.FAR) ? -LARGE_TURN : -SMALL_TURN);
                    deltaY = ((startPos == START_POS.FAR) ? WIGGLE : -WIGGLE);
                    INITIAL_ARM_EXTENTION = ((startPos == START_POS.FAR) ? 1262:398);
                }
                else {
                    dropAngle = ((startPos == START_POS.FAR) ? -35 : -LARGE_TURN);
                    deltaY = ((startPos == START_POS.FAR) ? -WIGGLE : WIGGLE);
                    INITIAL_ARM_EXTENTION = ((startPos == START_POS.FAR) ? 850:398);
                }
                break;
            default:
                deltaY = 0;
                dropAngle = 0;
                INITIAL_ARM_EXTENTION = 1262;
                break;
        }

        // Drop pose from unique starting positions
        dropPose = new Pose2d(FORWARD, deltaY, Math.toRadians(dropAngle));

        // Trajectories
        Trajectory start = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -220);
                })
                .lineToLinearHeading(dropPose)
                .addTemporalMarker(0.5, () -> {
                    super.arm.setArmPosition(1, INITIAL_ARM_EXTENTION);
                })
                .addTemporalMarker(1.5, () -> {
                    super.claw.leftOpen();
                })
                .addTemporalMarker(2.0, () -> {
                    super.arm.setArmPosition(1, 950);
                })
                .build();

        // Class Setup
        super.arm.setShoulder(shoulder);
        super.wrist.start();
        super.shoulder.start();
        super.claw.rightClose();
        super.claw.leftClose();
    }
}
