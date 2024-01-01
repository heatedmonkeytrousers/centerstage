package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name="Robot: Generic Automation", group="Robot")

public class AutonomousOpMode_Linear extends StandardSetupOpMode {
    public AutonomousOpMode_Linear() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Initial parking spot
        Motion.PARKING_SPOT parkingSpot = position;

        // Reset the 30 second runtime timer
        runtime.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        super.arm.setShoulder(shoulder);

        //super.arm.start();
        //super.shoulder.start();
        super.wrist.start();

        super.claw.rightClose();
        super.claw.leftClose();
        Trajectory start = drive.trajectoryBuilder(new Pose2d())

                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -200);
                })

                .lineToLinearHeading(new Pose2d(16.5, 0, Math.toRadians(0)))

                .addTemporalMarker(0.5, () -> {
                    super.arm.setArmPosition(1, 1650);
                })
                .addTemporalMarker(1.5, () -> {
                    super.claw.leftOpen();
                })




/*
                .addDisplacementMarker(() -> {
                    super.shoulder.setShoulderPosition(0.75, -655);
                    super.arm.setArmPosition(1, 2180);
                })
                .addDisplacementMarker(() -> {
                    super.claw.rightOpen();
                })
                .lineToSplineHeading(new Pose2d(-24, 0, Math.toRadians(25.5)))

 */
                .build();

        Trajectory board = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, -739);
                })
                .lineToSplineHeading(new Pose2d(25.5, -34, Math.toRadians(270)))
                .addTemporalMarker(1.5, () -> {
                    super.claw.rightOpen();
                })
                .build();


        Trajectory park = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 0);
                })
                .addTemporalMarker(0.5, () -> {
                    super.shoulder.setShoulderPosition(0.75, -20);
                })
                .lineToSplineHeading(new Pose2d(4, -30, Math.toRadians(90)))
                .build();
        // Wait to start autonomous
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(start);
        sleep(1500);
        drive.followTrajectory(board);
        sleep(1600);
        drive.followTrajectory(park);
        sleep(5000);

        // Determine parking spot
        parkingSpot = position;
        telemetry.addData("Parking Spot", parkingSpot);
        telemetry.update();




    }
}