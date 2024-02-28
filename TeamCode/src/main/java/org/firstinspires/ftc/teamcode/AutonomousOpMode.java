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

    private static final double MAX_POS_ERROR = 12.0; // Maximum position error (inches)
    private static final double MAX_HEADING_ERROR = Math.toRadians(20); // Max heading error (radians)

    private static final double FORWARD = 16.5;
    private static final double WIGGLE = 2.5;
    private static final double SMALL_TURN = 27;
    private static final double LARGE_TURN = 42;

    protected static final int CENTER_ARM_EXTENSION = 1300;
    protected int INITIAL_SHOULDER_RAISE = -150; //How high do we raise at the start
    protected int INITIAL_ARM_EXTENTION = 0; //How far the arm extends to drop the pixel on the tape

    protected SampleMecanumDrive drive;

    //poses
    protected Pose2d startPose = new Pose2d(0,0,0);
    protected Pose2d dropPose;
    protected START_POS startPos;

    // Trajectories
    //protected Trajectory start;

    // Scale applied to flip the y-axis
    protected double yScale;

    @Override
    public void runOpMode() throws InterruptedException {
        // Parent opmode call
        super.runOpMode();

        // Ensure claws are closed
        super.claw.leftClose();
        super.claw.rightClose();
    }

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

        // NEAR
        if(startPos == START_POS.NEAR)
        {
            if(color == COLOR.RED) {
                // NEAR/RED/LEFT
                if(hamsterPos == HAMSTER_POS.LEFT){
                    dropAngle = LARGE_TURN+1;
                    deltaY = -WIGGLE;
                    INITIAL_SHOULDER_RAISE = -100;
                    INITIAL_ARM_EXTENTION = 1200;
                }
                // NEAR/RED/CENTER
                else if(hamsterPos == HAMSTER_POS.CENTER){
                    dropAngle = 0;
                    deltaY = 0;
                    INITIAL_SHOULDER_RAISE = -200;
                    INITIAL_ARM_EXTENTION = CENTER_ARM_EXTENSION;
                }
                // NEAR/RED/RIGHT
                else{
                    dropAngle = -36;
                    deltaY = -WIGGLE;
                    INITIAL_SHOULDER_RAISE = -100;
                    INITIAL_ARM_EXTENTION = 820;
                }
            }
            else {
                // NEAR/BLUE/LEFT
                if(hamsterPos == HAMSTER_POS.LEFT){
                    dropAngle = SMALL_TURN;
                    deltaY = WIGGLE;
                    INITIAL_SHOULDER_RAISE = -100;
                    INITIAL_ARM_EXTENTION = 820;
                }
                // NEAR/BLUE/CENTER
                else if(hamsterPos == HAMSTER_POS.CENTER){
                    dropAngle = 0;
                    deltaY = 0;
                    INITIAL_SHOULDER_RAISE = -200;
                    INITIAL_ARM_EXTENTION = CENTER_ARM_EXTENSION;
                }
                // NEAR/BLUE/RIGHT
                else{
                    dropAngle = -50;
                    deltaY = WIGGLE;
                    INITIAL_SHOULDER_RAISE = -100;
                    INITIAL_ARM_EXTENTION = CENTER_ARM_EXTENSION-190;
                }
            }
        }
        // FAR
        else {
            if(color == COLOR.RED) {
                // FAR/RED/LEFT
                if(hamsterPos == HAMSTER_POS.LEFT){
                    dropAngle = SMALL_TURN;
                    deltaY = WIGGLE;
                    INITIAL_SHOULDER_RAISE = -100;
                    INITIAL_ARM_EXTENTION = 820;
                }
                // FAR/RED/CENTER
                else if(hamsterPos == HAMSTER_POS.CENTER){
                    dropAngle = 0;
                    deltaY = 0;
                    INITIAL_SHOULDER_RAISE = -200;
                    INITIAL_ARM_EXTENTION = CENTER_ARM_EXTENSION;
                }
                // FAR/RED/RIGHT
                else{
                    dropAngle = -50;
                    deltaY = WIGGLE;
                    INITIAL_SHOULDER_RAISE = -100;
                    INITIAL_ARM_EXTENTION = CENTER_ARM_EXTENSION-190;
                }
            }
            else {
                // FAR/BLUE/LEFT
                if(hamsterPos == HAMSTER_POS.LEFT){
                    dropAngle = LARGE_TURN+1;
                    deltaY = -WIGGLE;
                    INITIAL_SHOULDER_RAISE = -100;
                    INITIAL_ARM_EXTENTION = 1200;
                }
                // FAR/BLUE/CENTER
                else if(hamsterPos == HAMSTER_POS.CENTER){
                    dropAngle = 0;
                    deltaY = 0;
                    INITIAL_SHOULDER_RAISE = -200;
                    INITIAL_ARM_EXTENTION = CENTER_ARM_EXTENSION;
                }
                // FAR/BLUE/RIGHT
                else{
                    dropAngle = -36;
                    deltaY = -WIGGLE;
                    INITIAL_SHOULDER_RAISE = -100;
                    INITIAL_ARM_EXTENTION = 820;
                }
            }
        }

        // Drop pose from unique starting positions
        dropPose = new Pose2d(FORWARD, deltaY, Math.toRadians(dropAngle));

        // Class Setup
        super.arm.setShoulder(shoulder);
        super.shoulder.ignoreGamepad();
        super.arm.ignoreGamepad();
        super.claw.ignoreGamepad();
        super.wrist.start();
        super.shoulder.start();
        super.claw.rightClose();
        super.claw.leftClose();
    }


    /**
     * Checks the error of the last pose.  If it's really bad we probably hit something and we
     * should limp back home or just stay put.
     *
     * @return true if something is really off, false if we're ok
     */
    protected boolean checkError(Pose2d errorPose)
    {
        return errorPose.getX() > MAX_POS_ERROR || errorPose.getY() > MAX_POS_ERROR || errorPose.getHeading() > MAX_HEADING_ERROR;
    }
}
