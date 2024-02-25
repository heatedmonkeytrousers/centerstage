package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "Robot Setup Camera Super Class", group = "Robot")
@Disabled
public class CameraSetupOpMode extends LinearOpMode {

    // April Tag Setup (for 1080p camera)
    static final int IMAGE_WIDTH = 800;     // Camera can do up to 1920x1080
    static final int IMAGE_HEIGHT = 448;    // Camera can do up to 1920x1080
    static final double TAG_SIZE = 0.0508;  // 2in tag
    static final double FX = 578.272;       // Calibration for 800x448
    static final double FY = 578.272;       // Calibration for 800x448
    static final double CX = 402.145;       // Calibration for 800x448
    static final double CY = 221.506;       // Calibration for 800x448
    static final double INCHES_PER_METER = 39.3701;

    public static final int BLUE_LEFT_BOARD = 1;
    public static final int BLUE_CENTER_BOARD = 2;
    public static final int BLUE_RIGHT_BOARD = 3;
    public static final int BLUE_STACK_WALL = 9;
    public static final int BLUE_STACK_WALL_BIG = 10;

    public static final int RED_LEFT_BOARD = 4;
    public static final int RED_CENTER_BOARD = 5;
    public static final int RED_RIGHT_BOARD = 6;
    public static final int RED_STACK_WALL = 8;
    public static final int RED_STACK_WALL_BIG = 7;

    // Some typical values for April tag offset
    public static final double GRAB_DISTANCE = 11;
    public static final double DROP_DISTANCE = 12;
    public static final double RIGHT_DISTANCE = 0.5;
    public static final double LEFT_DISTANCE = -3.5;

    private boolean set = false;  // If this is false we continually look for the hamster
    private boolean pose = false; // If this is true we make one april tag measurement

    protected OpenCvWebcam webcam = null;
    protected AutonomousOpMode.COLOR color = AutonomousOpMode.COLOR.RED;
    protected AutonomousOpMode.HAMSTER_POS hamsterPos = AutonomousOpMode.HAMSTER_POS.CENTER;

    private int tag; // April tag to detect and act on
    private double targetX; // Distance left right from April tag (inches)
    private double targetZ; // Distance from April tag (inches)

    //Badger Bots ideal values
    static final Scalar RED = new Scalar(140, 23, 21);
    static final Scalar BLUE = new Scalar(5, 39, 80);
    static final Scalar GREY = new Scalar(140,140,140);
    static final int RED_THRESH = 50;
    static final int BLUE_THRESH = 50;

    static final int RED_REGION1_CAL = 0;
    static final int RED_REGION2_CAL = 0;
    static final int RED_REGION3_CAL = 0;
    static final int BLUE_REGION1_CAL = 0;
    static final int BLUE_REGION2_CAL = 0;
    static final int BLUE_REGION3_CAL = 0;

    public boolean red = false;

    /**
     * Method to stop the internal openCV loop from continuously detecting the hamster.
     */
    public void set(){
        set = true;
    }

    /**
     * Method to center an April tag in front of the robot.  We set the parameters for our final
     * position and then set a boolean to allow teh loop to run the april tag detection and
     * centering once.
     *
     * @param tag Tag number to look for (found robot will move, otherwise no movement)
     * @param awayOffset Distance in inches away from tag (11 inches is typical)
     * @param leftRightOffset Distance in inches left or right from tag (-3.5 lines up left)
     */
    public void aprilTagPose(int tag, double awayOffset, double leftRightOffset) {
        this.tag = tag;
        this.targetX = leftRightOffset;
        this.targetZ = awayOffset;
        pose = true;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Build the camera class
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new CameraCalibration());
        //webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                //Camera Starts Running
                webcam.startStreaming(IMAGE_WIDTH, IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //Run this Code if there is an Error
            }
        });
    }

    /**
     * OpenCvPielne does two things:
     *  1) Detects the hamster and determines which side it is on
     *  2) Detects an april tag and positions the robot in front of it
     */
    public class CameraCalibration extends OpenCvPipeline {
        private void colorDetermination(Mat input, AtomicInteger redCount, AtomicInteger blueCount) {
            int redTot = 0;
            int blueTot = 0;

            double redDist = 0;
            double blueDist = 0;

            //Goes through every pixel of input
            for (int r = 0; r < input.rows(); r++) {
                for (int c = 0; c < input.cols(); c++) {
                    // Get pixel RGB value
                    double[] v = input.get(r, c);

                    // Get distances
                    redDist = Math.sqrt(Math.pow((v[0] - RED.val[0]), 2) + Math.pow((v[1] - RED.val[1]), 2) + Math.pow((v[2] - RED.val[2]), 2));
                    blueDist = Math.sqrt(Math.pow((v[0] - BLUE.val[0]), 2) + Math.pow((v[1] - BLUE.val[1]), 2) + Math.pow((v[2] - BLUE.val[2]), 2));

                    // Increment counts if within threshold
                    if (redDist < blueDist) {
                        if (redDist < RED_THRESH) {
                            if (red) {
                                redTot++;
                            }
                        }
                    } else {
                        if (blueDist < BLUE_THRESH) {
                            if (!red) {
                                blueTot++;
                            }
                        }
                    }
                }
            }
            redCount.set(redTot);
            blueCount.set(blueTot);
        }

        private AprilTagDetectionPipeline.Pose aprilTagPoseToOpenCvPose(AprilTagPose aprilTagPose)
        {
            AprilTagDetectionPipeline.Pose pose = new AprilTagDetectionPipeline.Pose();
            pose.tvec.put(0,0, aprilTagPose.x);
            pose.tvec.put(1,0, aprilTagPose.y);
            pose.tvec.put(2,0, aprilTagPose.z);

            Mat R = new Mat(3, 3, CvType.CV_32F);

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    R.put(i,j, aprilTagPose.R.get(i,j));
                }
            }

            Calib3d.Rodrigues(R, pose.rvec);

            return pose;
        }

        private void moveRobot(AprilTagDetection detection) {
            if (detection != null) {
                double xPos = detection.pose.x * INCHES_PER_METER;
                double zPos = detection.pose.z * INCHES_PER_METER;
                double yPos = detection.pose.y * INCHES_PER_METER;
                Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                double yaw = -(Math.toRadians(rot.firstAngle));
                double minDist = 0.0;
                double deltaX = targetX - xPos;
                double deltaZ = zPos-targetZ;
                double dist = Math.sqrt(deltaX*deltaX + deltaZ*deltaZ);
                double minDegrees = Math.toRadians(0.0);

                if (dist > minDist || Math.abs(yaw) > minDegrees) {
                    // We are within range, need to move
                    SampleMecanumDrive SMD = new SampleMecanumDrive(hardwareMap);
                    Pose2d startPose = SMD.getPoseEstimate();
                    Pose2d endPose = new Pose2d(startPose.getX() + deltaZ, startPose.getY() + deltaX, startPose.getHeading() + yaw);
                    telemetry.addData("X pos", xPos);
                    telemetry.addData("Y pos", yPos);
                    telemetry.addData("Z pos", zPos);
                    telemetry.addData("First", rot.firstAngle);
                    telemetry.addData("Second", rot.secondAngle);
                    telemetry.addData("Third", rot.thirdAngle);telemetry.addData("Current X", startPose.getX());
                    telemetry.addData("Delta X", deltaX);
                    telemetry.addData("Current Y", startPose.getY());
                    telemetry.addData("Delta Y", deltaZ);
                    telemetry.addData("Current Heading", startPose.getHeading());
                    telemetry.addData("Yaw", yaw);
                    telemetry.update();

                    Trajectory trajectory = SMD.trajectoryBuilder(startPose)
                            .lineToLinearHeading(endPose,
                                    SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/3, DriveConstants.MAX_ANG_VEL/3, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/3))
                            .build();

                    SMD.followTrajectory(trajectory);
                }
            }
        }

        @Override
        public Mat processFrame(Mat input) {
            // Should we detect an april tag and then pose the robot in front of it?
            if(pose) {
                // Detect AprilTag
                Mat grey = new Mat();
                Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
                long nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
                ArrayList<AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, TAG_SIZE, FX, FY, CX, CY);
                for(AprilTagDetection dt : detections)
                {
                    if (dt.id == tag) {
                        moveRobot(dt);
                        break;
                    }
                }

                // Only detect and move the robot once
                // TODO - could run twice!
                pose = false;
            }

            // Have we already detected the hamster? (yes - then exit, no continue)
            if(set)return input;

            // Dynamic sizing
            Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,IMAGE_HEIGHT / 3);
            Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(IMAGE_WIDTH / 3,IMAGE_HEIGHT / 3);
            Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(IMAGE_WIDTH / 3 * 2,IMAGE_HEIGHT / 3);
            int REGION_WIDTH = IMAGE_WIDTH / 3;
            int REGION_HEIGHT = IMAGE_HEIGHT - IMAGE_HEIGHT /3;

            Point region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            Point region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            Point region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            Point region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            Point region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            Point region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            //Dividing our region into three sections using our points above
            Mat region1 = input.submat(new Rect(region1_pointA, region1_pointB));
            Mat region2 = input.submat(new Rect(region2_pointA, region2_pointB));
            Mat region3= input.submat(new Rect(region3_pointA, region3_pointB));

            //A test region to find the color values of an object
            //Mat test = input.submat(new Rect(160, 160, 10,10));

            //Gets the average of each color inside the test area
            //For finding the color value of objects
            //Scalar mean = Core.mean(test);

            //Creating vars to store the red and blue values for each of these regions
            AtomicInteger red1 = new AtomicInteger();
            AtomicInteger blue1 = new AtomicInteger();
            AtomicInteger red2 = new AtomicInteger();
            AtomicInteger blue2 = new AtomicInteger();
            AtomicInteger red3 = new AtomicInteger();
            AtomicInteger blue3 = new AtomicInteger();

            colorDetermination(region1, red1, blue1);
            colorDetermination(region2, red2, blue2);
            colorDetermination(region3, red3, blue3);

            // Remove calibration values from counts
            red1.addAndGet(-RED_REGION1_CAL);
            red2.addAndGet(-RED_REGION2_CAL);
            red3.addAndGet(-RED_REGION3_CAL);
            blue1.addAndGet(-BLUE_REGION1_CAL);
            blue2.addAndGet(-BLUE_REGION2_CAL);
            blue3.addAndGet(-BLUE_REGION3_CAL);

            //Use Math.max to determine if it is red or blue
            //Use nested if statements to determine which region it is in
            String colors;
            String region;
            int maxRed = Math.max(red1.get(), Math.max(red2.get(), red3.get()));
            int maxBlue = Math.max(blue1.get(), Math.max(blue2.get(), blue3.get()));
            if (maxRed > maxBlue) {
                //It is red
                color = AutonomousOpMode.COLOR.RED;
                colors = "red";
                if (red1.get() > red2.get() && red1.get() > red3.get()) {
                    hamsterPos = AutonomousOpMode.HAMSTER_POS.LEFT;
                    region = "left";
                } else if (red2.get() > red3.get()) {
                    hamsterPos = AutonomousOpMode.HAMSTER_POS.CENTER;
                    region = "center";
                } else {
                    hamsterPos = AutonomousOpMode.HAMSTER_POS.RIGHT;
                    region = "right";
                }
            } else {
                //It is blue
                colors = "blue";
                color = AutonomousOpMode.COLOR.BLUE;
                if (blue1.get() > blue2.get() && blue1.get() > blue3.get()) {
                    hamsterPos = AutonomousOpMode.HAMSTER_POS.LEFT;
                    region = "left";
                } else if (blue2.get() > blue3.get()) {
                    hamsterPos = AutonomousOpMode.HAMSTER_POS.CENTER;
                    region = "center";
                } else {
                    hamsterPos = AutonomousOpMode.HAMSTER_POS.RIGHT;
                    region = "right";
                }
            }

            //Displays rectangles for lining up the webcam
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    new Scalar(0, 0, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    new Scalar(0, 0, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    new Scalar(0, 0, 0), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //webcam.closeCameraDevice();
            /*
            Imgproc.rectangle(
                    input,
                    new Point(160, 160),
                    new Point(170, 170),
                    new Scalar(0,0,0),
                    1);

             */

            //  Debug
            telemetry.addData("Parking Spot", color);
            telemetry.addData("Red 1", red1.get());
            telemetry.addData("Red 2", red2.get());
            telemetry.addData("Red 3", red3.get());
            telemetry.addData("Blue 1", blue1.get());
            telemetry.addData("Blue 2", blue2.get());
            telemetry.addData("Blue 3", blue3.get());
            telemetry.addData("Color", colors);
            telemetry.addData("Region", region);
            //telemetry.addData("Mean", "%d %d %d", (int) mean.val[0], (int) mean.val[1], (int) mean.val[2]);
            telemetry.update();
            return input;
        }
    }
}