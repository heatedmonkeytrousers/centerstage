package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class AutonomousNearBoard extends AutonomousOpMode {
    public AutonomousNearBoard() {
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
        Pose2d cyclePose = new Pose2d(5, 69 * -yScale, Math.toRadians(-90 * yScale));
        Pose2d grabPose = new Pose2d(32.5,68 * -yScale, Math.toRadians(-90 * yScale));
        Pose2d grab2Pose = new Pose2d(28,68 * -yScale, Math.toRadians(-90 * yScale));
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

        Trajectory stack = drive.trajectoryBuilder(parkPose)
                .lineToLinearHeading(cyclePose)
                .build();

        Trajectory grab1 = drive.trajectoryBuilder(cyclePose)
                .lineToLinearHeading(grabPose)
                .build();

        Trajectory grab2 = drive.trajectoryBuilder(grabPose)
                .lineToLinearHeading(grab2Pose)
                .build();

        Trajectory corner = drive.trajectoryBuilder(grab2Pose)
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
        drive.followTrajectory(stack);
        shoulder.setShoulderPosition(0.5,-200);
        drive.followTrajectory(grab1);
        arm.setArmPosition(0.5, 490);
        sleep(200);
        if (color == COLOR.BLUE)
            claw.rightClose();
        else
            claw.leftClose();
        sleep(1000);
        shoulder.setShoulderPosition(0.5,-240);
        sleep(400);
        drive.followTrajectory(grab2);
        shoulder.setShoulderPosition(0.5,-160);
        sleep(200);
       if (color == COLOR.BLUE )
           claw.leftClose();
       else
           claw.rightClose();
        sleep(1000);
        arm.setArmPosition(1.0,0);
        sleep(400);
        drive.followTrajectory(corner);
        shoulder.setShoulderPosition(0.5, -40);
        drive.followTrajectory(back);
        drive.followTrajectory(board2);
        arm.setArmPosition(1.0, -800);
        shoulder.setShoulderPosition(0.4, -2100);
        sleep(700);
        claw.leftOpen();
        claw.rightOpen();
        sleep(600);
        arm.setArmPosition(1.0, 0);
        shoulder.setShoulderPosition(0.8,0);
        sleep(2000);

    }
}