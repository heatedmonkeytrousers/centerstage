package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class AutonomousFarFromBoard extends AutonomousOpMode {
    private double PARTNER_WAIT_SECONDS = 5.0;

    public AutonomousFarFromBoard() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup
        super.runOpMode();

        // Wait to start autonomous
        waitForStart();
        super.set();
        super.setup(hardwareMap, START_POS.FAR, hamsterPos, color);

        // Robot Poses
        Pose2d avoidPose = new Pose2d(14, yScale * -23, Math.toRadians(90 * yScale));
        Pose2d alignPose = new Pose2d(52, yScale * -19, Math.toRadians(90 * yScale));
        Pose2d backPose = new Pose2d(50, yScale * 81, Math.toRadians(90 * yScale));
        double boardDropX = (hamsterPos == HAMSTER_POS.LEFT) ? 25-(7 * yScale): (hamsterPos == HAMSTER_POS.RIGHT) ? 25+(6 * yScale): 25;
        Pose2d releasePose = new Pose2d(boardDropX, yScale * 84, Math.toRadians(90 * yScale));
        Pose2d boardBackUp = new Pose2d(boardDropX, yScale * 81, Math.toRadians(90 * yScale));
        Pose2d parkPose = new Pose2d(32, yScale * 82, Math.toRadians(-90 * yScale));

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
                .build();

        Trajectory back = drive.trajectoryBuilder(dropPose)
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1.0, 0);
                })
                .addTemporalMarker(0.5, () -> {
                    super.shoulder.setShoulderPosition(0.75, -100);
                })
                .lineToLinearHeading(avoidPose)
                .addTemporalMarker(1, () -> {
                    super.shoulder.setShoulderPosition(0.75, -50);
                })
                .build();

        Trajectory slide = drive.trajectoryBuilder(avoidPose)
                .lineToLinearHeading(alignPose)
                .build();

        Trajectory forward = drive.trajectoryBuilder(alignPose)
                .lineToLinearHeading(backPose)
                .build();

        Trajectory board = drive.trajectoryBuilder(backPose)
                .lineToLinearHeading(releasePose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.5, -520);
                })
                .addTemporalMarker(1, () -> {
                    super.arm.setArmPosition(1, 800);
                })
                .addTemporalMarker(2, () -> {
                    super.claw.rightOpen();
                })
                .build();

        Trajectory backUp = drive.trajectoryBuilder(releasePose)
                .lineToLinearHeading(boardBackUp)
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 0);
                })
                .build();

        Trajectory park = drive.trajectoryBuilder(boardBackUp)
                .lineToLinearHeading(parkPose)
                .addTemporalMarker(1, () -> {
                    super.shoulder.setShoulderPosition(0.5, -40);
                })
                .build();
        if (isStopRequested()) return;

        // Initial drop, drive to board and drop then park
        drive.followTrajectory(start);
        sleep(1000);
        drive.followTrajectory(back);
        drive.followTrajectory(slide);
        drive.followTrajectory(forward);
        sleep((int)(PARTNER_WAIT_SECONDS*1000));
        drive.followTrajectory(board);
        sleep(700);
        drive.followTrajectory(backUp);
        sleep(500);
        drive.followTrajectory(park);
    }
}