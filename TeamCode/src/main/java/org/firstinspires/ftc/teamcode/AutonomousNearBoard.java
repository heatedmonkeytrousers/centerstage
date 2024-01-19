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
        // Get position and color from camera
        //HAMSTER_POS hamsterPos = HAMSTER_POS.RIGHT;
        //COLOR color = COLOR.BLUE;

        //Setup
        super.runOpMode();
        super.sleep(3000);
        super.setup(hardwareMap, START_POS.NEAR, hamsterPos, color);

        // Poses
        double boardDropX = (hamsterPos == HAMSTER_POS.LEFT) ? 26.5-(6 * yScale): (hamsterPos == HAMSTER_POS.RIGHT) ? 26.5+(4 * yScale): 26.5;
        Pose2d boardPose = new Pose2d(boardDropX, 35 * yScale, Math.toRadians((360 + (yScale * 90)) % 360));
        Pose2d parkPose = new Pose2d(4, 33 * yScale, Math.toRadians(-90 * yScale));

        // Trajectories
        Trajectory start = drive.trajectoryBuilder(startPose)

                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.75, INITIAL_SHOULDER_RAISE);
                })

                .lineToLinearHeading(dropPose)
                .addTemporalMarker(0.5, () -> {
                    super.arm.setArmPosition(0.5, INITIAL_ARM_EXTENTION);
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
        Trajectory board = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.5, -550);
                })
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 950);
                })
                .lineToSplineHeading(boardPose)
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
                .lineToSplineHeading(parkPose)
                .build();

        // Wait to start autonomous
        waitForStart();
        if (isStopRequested()) return;

        // Initial drop, drive to board and drop then park
        drive.followTrajectory(start);
        sleep(1000);
        drive.followTrajectory(board);
        sleep(200);
        drive.followTrajectory(park);
    }
}