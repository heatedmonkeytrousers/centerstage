package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Autonomous: Far From Board", group="Robot")

public class AutonomousFarFromBoard extends AutonomousOpMode {

    protected static final double LEFT_WIGGLE = -20;
    protected static final double RIGHT_WIGGLE = 38;
    public AutonomousFarFromBoard() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        //If red alliance then yScale is -1.0
        //If blue alliance then yScale is 1.0
        double yScale = -1.0;

        // Reset the 30 second runtime timer
        runtime.reset();

        super.setup(hardwareMap);
        super.arm.setShoulder(shoulder);
        super.wrist.start();
        super.claw.rightClose();
        super.claw.leftClose();

        // Robot Poses
        Pose2d dropPose = new Pose2d(16.5, 0, Math.toRadians(RIGHT_WIGGLE * yScale));
        Pose2d avoidPose = new Pose2d(14,yScale * -19, Math.toRadians(90 * yScale));
        Pose2d alignPose = new Pose2d(53,yScale * -19, Math.toRadians(90 * yScale));
        Pose2d backPose = new Pose2d(53,yScale * 85, Math.toRadians(90 * yScale));
        Pose2d releasePose = new Pose2d(25,yScale * 86, Math.toRadians(90 * yScale));
        Pose2d parkPose = new Pose2d(30,yScale * 85, Math.toRadians(-90 * yScale));

        Trajectory start = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -220);
                })
                .lineToLinearHeading(dropPose)
                .addTemporalMarker(0.5, () -> {
                    super.arm.setArmPosition(1, 1630);
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
                    super.shoulder.setShoulderPosition(0.75, -600);
                })
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 950);
                })
                .addTemporalMarker(2, () -> {
                    super.claw.rightOpen();
                })
                .build();

        Trajectory park = drive.trajectoryBuilder(releasePose)
                .lineToLinearHeading(parkPose)
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 0);
                })
                .addTemporalMarker(0.5, () -> {
                    super.shoulder.setShoulderPosition(0.75, -40);
                })
                .build();

        // Wait to start autonomous
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(start);
        sleep(1000);
        drive.followTrajectory(back);
        drive.followTrajectory(forward1);
        drive.followTrajectory(forward2);
        drive.followTrajectory(board);
        sleep(500);
        drive.followTrajectory(park);
    }
}