package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Autonomous: Near Board", group = "Robot")

public class AutonomousNearBoard extends AutonomousOpMode {
    public AutonomousNearBoard() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Setup
        super.runOpMode();
        super.setup(hardwareMap, START_POS.NEAR, HAMSTER_POS.RIGHT, COLOR.RED);

        // Poses
        Pose2d boardPose = new Pose2d(25.5, 35 * yScale, Math.toRadians((360 + (yScale * 90)) % 360));
        Pose2d parkPose = new Pose2d(4, 33 * yScale, Math.toRadians(-90 * yScale));

        // Trajectories
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

        // Setting initial pose to startPose makes the robot avoid the drop areas
        Trajectory board = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -600);
                })
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 950);
                })
                .lineToSplineHeading(boardPose)
                .addTemporalMarker(2, () -> {
                    super.claw.rightOpen();
                })
                .build();


        Trajectory park = drive.trajectoryBuilder(boardPose)
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 0);
                })
                .addTemporalMarker(0.5, () -> {
                    super.shoulder.setShoulderPosition(0.75, -40);
                })
                .lineToSplineHeading(parkPose)
                .build();

        // Wait to start autonomous
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(start);
        sleep(1000);
        drive.followTrajectory(board);
        sleep(200);
        drive.followTrajectory(park);
        sleep(5000);

    }
}