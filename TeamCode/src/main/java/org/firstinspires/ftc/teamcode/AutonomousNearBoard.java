package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Autonomous: Near", group="Robot")

public class AutonomousNearBoard extends StandardSetupOpMode {
    public AutonomousNearBoard() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Initial parking spot
        Motion.PARKING_SPOT parkingSpot = position;

        // Determine parking spot
        parkingSpot = position;
        telemetry.addData("Parking Spot", parkingSpot);
        telemetry.update();

        //If red alliance then yScale is -1.0
        //If blue alliance then yScale is 1.0
        double yScale = -1.0;

        // Reset the 30 second runtime timer
        runtime.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        super.arm.setShoulder(shoulder);
        super.wrist.start();
        super.claw.rightClose();
        super.claw.leftClose();

        // Poses
        Pose2d startPose = new Pose2d();
        Pose2d dropPose = new Pose2d(16.5, 0, 0);
        Pose2d boardPose = new Pose2d(25.5, 34 * yScale, Math.toRadians((360+(yScale*90)) % 360));
        Pose2d parkPose = new Pose2d(4, 30 * yScale, Math.toRadians(-90 * yScale));

        // Trajectories
        Trajectory start = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -200);
                })
                .lineToLinearHeading(dropPose)
                .addTemporalMarker(0.5, () -> {
                    super.arm.setArmPosition(1, 1650);
                })
                .addTemporalMarker(1.5, () -> {
                    super.claw.leftOpen();
                })
                .build();

        // Setting initial pose to startPose makes the robot avoid the drop areas
        Trajectory board = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -739);
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
                    super.shoulder.setShoulderPosition(0.75, 0);
                })
                .lineToSplineHeading(parkPose)
                .build();

        // Wait to start autonomous
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(start);
        sleep(1000);
        drive.followTrajectory(board);
        sleep(200);
        drive.followTrajectory(park);
        sleep(5000);

    }
}