package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Autonomous: Far From Board", group = "Robot")

public class AutonomousFarFromBoard extends AutonomousOpMode {

    public AutonomousFarFromBoard() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Get position and color from camera
        //HAMSTER_POS hamsterPos = HAMSTER_POS.RIGHT;
        //COLOR color = COLOR.BLUE;

        // Setup
        super.runOpMode();
        super.sleep(4000);
        super.setup(hardwareMap, START_POS.FAR, hamsterPos, color);

        // Robot Poses
        Pose2d avoidPose = new Pose2d(14, yScale * -23, Math.toRadians(90 * yScale));
        Pose2d alignPose = new Pose2d(53, yScale * -19, Math.toRadians(90 * yScale));
        Pose2d backPose = new Pose2d(53, yScale * 83, Math.toRadians(90 * yScale));
        double boardDropX = (hamsterPos == HAMSTER_POS.LEFT) ? 25-(7 * yScale): (hamsterPos == HAMSTER_POS.RIGHT) ? 25+(6 * yScale): 25;
        Pose2d releasePose = new Pose2d(boardDropX, yScale * 83, Math.toRadians(90 * yScale));
        Pose2d boardBackUp = new Pose2d(boardDropX, yScale * 81, Math.toRadians(90 * yScale));
        Pose2d parkPose = new Pose2d(32, yScale * 82, Math.toRadians(-90 * yScale));

        Trajectory start = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -200);
                })
                .lineToLinearHeading(dropPose)
                .addTemporalMarker(0.5, () -> {
                    super.arm.setArmPosition(1, INITIAL_ARM_EXTENTION);
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

        Trajectory forward1 = drive.trajectoryBuilder(avoidPose)
                .lineToLinearHeading(alignPose)
                .build();

        Trajectory forward2 = drive.trajectoryBuilder(alignPose)
                .lineToLinearHeading(backPose)
                .build();

        Trajectory board = drive.trajectoryBuilder(backPose)
                .lineToLinearHeading(releasePose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -480);
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
                    super.shoulder.setShoulderPosition(0.75, -40);
                })
                .build();

        // Wait to start autonomous
        waitForStart();
        if (isStopRequested()) return;

        // Initial drop, drive to board and drop then park
        drive.followTrajectory(start);
        sleep(1000);
        drive.followTrajectory(back);
        drive.followTrajectory(forward1);
        drive.followTrajectory(forward2);
        drive.followTrajectory(board);
        sleep(700);
        drive.followTrajectory(backUp);
        sleep(500);
        drive.followTrajectory(park);
    }
}