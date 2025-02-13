package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutonomousFarFromBoard extends AutonomousOpMode {
    protected boolean oldOverride = false;
    protected double partnerWaitSeconds = 0.0;

    public AutonomousFarFromBoard() {
    }

    private int getBoardAprilTag() {
        int aprilTag;
        if (red) {
            aprilTag = (hamsterPos == HAMSTER_POS.LEFT) ? RED_LEFT_BOARD : (hamsterPos == HAMSTER_POS.CENTER) ?
                    RED_CENTER_BOARD : RED_RIGHT_BOARD;
        } else {
            aprilTag = (hamsterPos == HAMSTER_POS.LEFT) ? BLUE_LEFT_BOARD : (hamsterPos == HAMSTER_POS.CENTER) ?
                    BLUE_CENTER_BOARD : BLUE_RIGHT_BOARD;
        }
        return aprilTag;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup
        super.runOpMode();

        // Wait to start autonomous
        waitForStart();
        super.set();
        super.setup(hardwareMap, START_POS.FAR, hamsterPos, color);

        // Old Poses (we're hitting the farthest pixel trying to pick up from the stack, so don't)
        Pose2d oldAvoidPose = new Pose2d(14, yScale * -23, Math.toRadians(90 * yScale));
        Pose2d oldAlignPose = new Pose2d(52, yScale * -19, Math.toRadians(90 * yScale));
        boolean runOld = oldOverride ||
                (red && (hamsterPos == HAMSTER_POS.LEFT) ) ||
                (!red && (hamsterPos == HAMSTER_POS.RIGHT));
        if(runOld && partnerWaitSeconds>0) partnerWaitSeconds += 5.0;

        // Robot Poses
        //Pose2d avoidPose = new Pose2d(14, yScale * -21, Math.toRadians(-90 * yScale));
        Pose2d grabPose = new Pose2d(27,yScale * -21, Math.toRadians(-90 * yScale));
        Pose2d alignPose = new Pose2d(52, yScale * -21, Math.toRadians(-90 * yScale));
        Pose2d backPose = new Pose2d(50, yScale * 80, Math.toRadians((runOld?90:-90) * yScale));
        double boardDropX = (hamsterPos == HAMSTER_POS.LEFT)? 27-(7 * yScale): (hamsterPos == HAMSTER_POS.RIGHT) ? 27+(6 * yScale): 26;
        Pose2d releasePose = new Pose2d(boardDropX, yScale * 85, Math.toRadians(90 * yScale));
        Pose2d boardBackUp = new Pose2d(boardDropX, yScale * 81, Math.toRadians(90 * yScale));
        Pose2d parkPose = new Pose2d(50, yScale * 82, Math.toRadians(-90 * yScale));

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

        //Trajectory avoid = drive.trajectoryBuilder(dropPose)
        //        .addTemporalMarker(0, () -> {
        //            super.arm.setArmPosition(1.0, 0);
        //        })
        //        .addTemporalMarker(0.5, () -> {
        //            super.shoulder.setShoulderPosition(0.75, -100);
        //        })
        //        .lineToLinearHeading(avoidPose)
        //        .addTemporalMarker(1, () -> {
        //            super.shoulder.setShoulderPosition(0.75, -75);
        //        })
        //        .build();

        Trajectory grab = drive.trajectoryBuilder(dropPose)
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1.0, 0);
                })
                .addTemporalMarker(0.5, () -> {
                    super.shoulder.setShoulderPosition(0.75, -100);
                })
                .lineToLinearHeading(grabPose)
                .addTemporalMarker(1, () -> {
                    super.shoulder.setShoulderPosition(0.75, -75);
                })
                .build();

        Trajectory forward = drive.trajectoryBuilder(alignPose)
                .lineToLinearHeading(backPose)
                .build();

        Trajectory board = drive.trajectoryBuilder(backPose)
                .lineToLinearHeading(releasePose)//,
                        //SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/2, DriveConstants.MAX_ANG_VEL/2, DriveConstants.TRACK_WIDTH),
                        //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .addTemporalMarker(0, () -> {
                    super.shoulder.setShoulderPosition(0.5, -575);
                })
                .addTemporalMarker(1, () -> {
                    super.arm.setArmPosition(0.9, 1000);
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
        sleep(500);
        if(runOld) // To avoid hitting things and messing up the run
        {
            Trajectory oldAvoid = drive.trajectoryBuilder(dropPose)
                    .addTemporalMarker(0, () -> {
                        super.arm.setArmPosition(1.0, 0);
                    })
                    .addTemporalMarker(0.5, () -> {
                        super.shoulder.setShoulderPosition(0.75, -100);
                    })
                    .lineToLinearHeading(oldAvoidPose)
                    .addTemporalMarker(1, () -> {
                        super.shoulder.setShoulderPosition(0.75, -50);
                    })
                    .build();
            Trajectory oldSlide = drive.trajectoryBuilder(oldAvoidPose)
                    .lineToLinearHeading(oldAlignPose)
                    .build();
            Trajectory oldForward = drive.trajectoryBuilder(oldAlignPose)
                    .lineToLinearHeading(backPose)
                    .build();
            drive.followTrajectory(oldAvoid);
            drive.followTrajectory(oldSlide);
            drive.followTrajectory(oldForward);
        }
        else {
            //drive.followTrajectory(avoid);
            drive.followTrajectory(grab);
            aprilTagPose((red) ? RED_STACK_WALL : BLUE_STACK_WALL, GRAB_DISTANCE, LEFT_DISTANCE);
            shoulder.setShoulderPosition(0.7, -170);
            sleep(1200);
            arm.setArmPosition(0.5, 350);
            sleep(400);
            claw.leftClose();
            shoulder.setShoulderPosition(0.7, -200);
            sleep(1500);
            arm.setArmPosition(1, 0);
            sleep(500);
            Trajectory slide = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(alignPose,
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 2, DriveConstants.MAX_ANG_VEL / 2, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                    .addTemporalMarker(0, () -> {
                        super.shoulder.setShoulderPosition(0.5, -75);
                    })
                    .build();
            drive.followTrajectory(slide);
            drive.followTrajectory(forward);
        }
        sleep((int)(partnerWaitSeconds*1000));
        drive.followTrajectory(board);
        aprilTagPose(getBoardAprilTag(), DROP_DISTANCE, RIGHT_DISTANCE);
        sleep(1500);
        claw.rightOpen();
        sleep(500);
        claw.leftOpen();
        sleep(500);
        Trajectory backUp = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(boardBackUp)
                .addTemporalMarker(0, () -> {
                    super.arm.setArmPosition(1, 0);
                })
                .build();
        drive.followTrajectory(backUp);
        sleep(500);
        drive.followTrajectory(park);
        sleep(2000);
    }
}