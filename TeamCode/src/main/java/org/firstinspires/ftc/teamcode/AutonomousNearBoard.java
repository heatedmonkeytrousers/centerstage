package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.WriteAbortedException;

public class AutonomousNearBoard extends AutonomousOpMode {
    public AutonomousNearBoard() {
    }

    protected boolean cycle = true;

    /**
     * Detect if we hit the truss and recover if possible
     * @param parkPose pose we should park at
     * @return true if we hit the truss
     */
    private boolean didWeHitTheTruss(Pose2d parkPose)
    {
        boolean weHit = checkError(drive.getLastError());
        if(weHit)
        {
            // We hit the truss, try to limp back home
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d recoverPose = new Pose2d(5, currentPose.getY(), Math.toRadians(-90 * yScale));
            Trajectory recover = drive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(recoverPose,
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/3, DriveConstants.MAX_ANG_VEL/3, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/3))
                    .build();
            Trajectory safePark = drive.trajectoryBuilder(recoverPose)
                    .lineToLinearHeading(parkPose,
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2, DriveConstants.MAX_ANG_VEL/2, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                    .build();
            drive.followTrajectory(recover);
            drive.followTrajectory(safePark);

            // Put the arm in a safe spot for tele-op and wait a little bit
            arm.setArmPosition(1.0, 0);
            shoulder.setShoulderPosition(0.8,0);
            sleep(2000);
        }
        return weHit;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Setup
        super.runOpMode();

        // Wait to start autonomous
        waitForStart();
        super.set();
        super.setup(hardwareMap, START_POS.NEAR, hamsterPos, color);

        // Poses
        double boardDropX = (hamsterPos == HAMSTER_POS.LEFT) ? 26.5-(6 * yScale): (hamsterPos == HAMSTER_POS.RIGHT) ? 26.5+(4 * yScale): 26.5;
        Pose2d boardPose = new Pose2d(boardDropX, 35 * yScale, Math.toRadians((360 + (yScale * 90)) % 360));
        Pose2d parkPose = new Pose2d(5, 33 * yScale, Math.toRadians(-90 * yScale));
        Pose2d deepParkPose = new Pose2d(5, 39 * yScale, Math.toRadians(-90 * yScale));
        Pose2d cyclePose = new Pose2d(5, 69 * -yScale, Math.toRadians(-90 * yScale));
        Pose2d grabPose = new Pose2d(28,68 * -yScale, Math.toRadians(-90 * yScale)); //32.5
        //Pose2d grab2Pose = new Pose2d(28,68 * -yScale, Math.toRadians(-90 * yScale));
        Pose2d backwardBoardPose = new Pose2d(boardDropX, 35 * yScale, Math.toRadians(-90 *yScale));

        // Trajectories
        Trajectory start = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.5, INITIAL_SHOULDER_RAISE);
                })
                .lineToLinearHeading(dropPose)
                .addTemporalMarker(0.5, () -> {
                    super.arm.setArmPosition(0.75, INITIAL_ARM_EXTENTION);
                })
                .addTemporalMarker(1.5, () -> {
                    super.claw.leftOpen();
                })
                .addTemporalMarker(2.5, () -> {
                    super.arm.setArmPosition(0.5, INITIAL_ARM_EXTENTION-100);
                })
                .addTemporalMarker(2.0, () -> {
                    super.shoulder.setShoulderPosition(0.5, -200);
                })
                .build();

        // Setting initial pose to startPose makes the robot avoid the drop areas
        Trajectory board = drive.trajectoryBuilder(dropPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.5, -500);
                })
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 970);
                })
                .lineToLinearHeading(boardPose)
                .addTemporalMarker(2.5, () -> {
                    super.claw.rightOpen();
                })
                .build();


        Trajectory park = drive.trajectoryBuilder(boardPose)
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 0);
                })
                .addTemporalMarker(0.5, () -> {
                    super.shoulder.setShoulderPosition(0.5, -40);
                })
                .lineToLinearHeading(parkPose)
                .build();

        Trajectory deepPark = drive.trajectoryBuilder(parkPose)
                .lineToLinearHeading(deepParkPose)
                .build();

        Trajectory stack = drive.trajectoryBuilder(parkPose)
                .lineToLinearHeading(cyclePose)
                .build();

        Trajectory grab = drive.trajectoryBuilder(cyclePose)
                .lineToLinearHeading(grabPose)
                .build();

        Trajectory corner = drive.trajectoryBuilder(grabPose)
                .lineToLinearHeading(cyclePose)
                .build();

        Trajectory back = drive.trajectoryBuilder(cyclePose)
                .lineToLinearHeading(parkPose)
                .build();

        Trajectory board2 = drive.trajectoryBuilder(parkPose)
                .lineToLinearHeading(backwardBoardPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.5, -1800);
                })
                .build();

        Trajectory back2 = drive.trajectoryBuilder(backwardBoardPose)
                .lineToLinearHeading(parkPose)
                .build();

        if (isStopRequested()) return;

        // Initial drop, drive to board and drop then park
        drive.followTrajectory(start);
        sleep(500);
        drive.followTrajectory(board);
        sleep(500);
        drive.followTrajectory(park);
        if(cycle) {
            drive.followTrajectory(stack);
            // Did we make it through the truss!
            if (didWeHitTheTruss(parkPose)) return;
            drive.followTrajectory(grab);
            aprilTagPose((red) ? RED_STACK_WALL : BLUE_STACK_WALL, GRAB_DISTANCE, LEFT_DISTANCE);
            shoulder.setShoulderPosition(0.7, -170);
            sleep(800);
            arm.setArmPosition(0.5, 350);
            sleep(400);
            // Maybe we'll get lucky and grab two with one grab!
            shoulder.setShoulderPosition(0.7, -160);
            claw.leftClose();
            sleep(1200);
            // 2nd grab is taking too long and is not working
            //shoulder.setShoulderPosition(0.5,-575);
            //aprilTagPose((red)? RED_STACK_WALL : BLUE_STACK_WALL, GRAB_DISTANCE, RIGHT_DISTANCE);
            //sleep(500);
            //shoulder.setShoulderPosition(0.5,-140);
            //sleep(600);
            //claw.rightClose();
            //sleep(1200);
            arm.setArmPosition(1.0, 0);
            sleep(500);
            drive.followTrajectory(corner);
            shoulder.setShoulderPosition(0.5, -40);
            drive.followTrajectory(back);
            // Did we make it through the truss!
            if (didWeHitTheTruss(parkPose)) return;
            drive.followTrajectory(board2);
            arm.setArmPosition(1.0, -800);
            shoulder.setShoulderPosition(0.4, -2100);
            sleep(700);
            claw.leftOpen();
            //claw.rightOpen();
            sleep(600);
        }
        else
        {
            drive.followTrajectory(deepPark);
        }
        arm.setArmPosition(1.0, 0);
        shoulder.setShoulderPosition(0.8,0);
        sleep(2000);
    }
}