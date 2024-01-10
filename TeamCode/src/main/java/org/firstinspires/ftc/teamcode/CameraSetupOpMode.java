package org.firstinspires.ftc.teamcode;

import android.bluetooth.le.ScanCallback;
import android.util.MutableInt;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "Robot Setup Camera Super Class", group = "Robot")
@Disabled
public class CameraSetupOpMode extends LinearOpMode {

    protected OpenCvWebcam webcam = null;
    protected Scalar mu = new Scalar(0, 0, 0);
    protected Motion.COLOR color = Motion.COLOR.RED;


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
                //1920, 1080
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //Run this Code if there is an Error
            }
        });
    }

    public class CameraCalibration extends OpenCvPipeline {

        //Sets up position var and output
        Mat output = new Mat();

        private void colorDetermination(Mat input, AtomicInteger redCount, AtomicInteger blueCount) {
            int redTot = 0;
            int blueTot = 0;
            int greenTot = 0;

            double redDist = 0;
            double blueDist = 0;
            double greyDist = 0;

            //Badger Bots ideal values
            Scalar red = new Scalar(90, 42, 72);
            Scalar blue = new Scalar(25, 70, 123);
            Scalar green = new Scalar(55, 101, 88);
            Scalar grey = new Scalar(140,140,140);

            Mat mask = new Mat(input.rows() , input.cols(), input.type());
            int count = 0;

            //Goes through every pixel of input
            for (int r = 0; r < input.rows(); r++) {
                for (int c = 0; c < input.cols(); c++) {
                    double[] v = input.get(r, c);
                    //Gets averages for each color

                    redDist = Math.sqrt(Math.pow((v[0] - red.val[0]), 2) + Math.pow((v[1] - red.val[1]), 2) + Math.pow((v[2] - red.val[2]), 2));
                    blueDist = Math.sqrt(Math.pow((v[0] - blue.val[0]), 2) + Math.pow((v[1] - blue.val[1]), 2) + Math.pow((v[2] - blue.val[2]), 2));

                    if (redDist < blueDist) {
                        if (redDist < 40) {
                            redTot++;
                        }
                    } else {
                        if (blueDist < 50) {
                            blueTot++;
                        }
                    }

                }
            }

            redCount.set(redTot);
            blueCount.set(blueTot);
        }


        @Override
        public Mat processFrame(Mat input) {
            output = input;
            int buffer = 30;

            Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,80);
            Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(106-buffer,80);
            Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(212,80);
            int REGION_WIDTH = 106;
            int REGION_HEIGHT = 100;

            Point region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            Point region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH-buffer,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            Point region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            Point region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH+buffer,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            Point region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            Point region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            Mat region1 = input.submat(new Rect(region1_pointA, region1_pointB));
            Mat region2 = input.submat(new Rect(region2_pointA, region2_pointB));
            Mat region3= input.submat(new Rect(region3_pointA, region3_pointB));

            Mat test = input.submat(new Rect(160, 160, 10,10));

            double count = 0;
            double redTot = 0;
            double greenTot = 0;
            double blueTot = 0;
            for (int r = 0; r < test.rows(); r++) {
                for (int c = 0; c < test.cols(); c++) {
                    double[] v = test.get(r, c);
                    //Gets averages for each color
                        //Increments the total of red, green, and blue pixels
                        count++;
                        redTot += v[0];
                        greenTot += v[1];
                        blueTot += v[2];

                }
            }
            double averageRed = redTot / count;
            double averageGreen = greenTot / count;
            double averageBlue = blueTot / count;

            AtomicInteger red1 = new AtomicInteger();
            AtomicInteger blue1 = new AtomicInteger();
            AtomicInteger red2 = new AtomicInteger();
            AtomicInteger blue2 = new AtomicInteger();
            AtomicInteger red3 = new AtomicInteger();
            AtomicInteger blue3 = new AtomicInteger();

            colorDetermination(region1, red1, blue1);
            colorDetermination(region2, red2, blue2);
            colorDetermination(region3, red3, blue3);



            //Red val w/o Hamster 1
            int initRed1 = 0;
            //red1.set(red1.get() - initRed1);
            //Red val w/o Hamster 2
            int initRed2 = 0;
            //red2.set(red2.get() - initRed2);
            //Red val w/o Hamster 3
            int initRed3 = 11;
            //red3.set(red3.get() - initRed3);

            //Blue val w/o Hamster 1
            int initBlue1;
            //Blue val w/o Hamster 2
            int initBlue2;
            //Blue val w/o Hamster 3
            int initBlue3;

            //Use Math.max to determine if it is red or blue
            //Use nested if statements to determine which region it is in
            String colors;
            String region;
            if (Math.max(red1.get(), Math.max(red2.get(), red3.get())) > Math.max(blue1.get(), Math.max(blue2.get(), blue3.get()))) {
                //It is red
                if (red1.get() > red2.get() && red1.get() > red3.get()) {
                    colors = "red";
                    region = "1";
                    //It is in region 1
                } else if (red2.get() > red3.get()) {
                    region = "2";
                    colors = "red";
                    //It is in region 2
                } else {
                    colors = "red";
                    region = "3";
                    //It is in region 3
                }
            } else {
                //It is blue
                if (blue1.get() > blue2.get() && blue1.get() > blue3.get()) {
                    colors = "blue";
                    region = "1";
                    //It is in region 1
                } else if (blue2.get() > blue3.get()) {
                    colors = "blue";
                    region = "2";
                    //It is in region 2
                } else {
                    colors = "blue";
                    region = "3";
                    //It is in region 3
                }
            }

            //Displays a rectangle for lining up the webcam
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

            Imgproc.rectangle(
                    input,
                    new Point(100, 100),
                    new Point(110, 110),
                    new Scalar(0,0,0),
                    1);
/*
            Imgproc.rectangle(
                    output,
                    new Point (
                            input.cols()/3,
                            input.rows()/2.5),
                    new Point(
                            input.cols()*(3f/6f),
                            input.rows()*(3f/5f)),
                    new Scalar(0, 255, 0), 1);

 */




            //webcam.closeCameraDevice();

            //  Debug
            telemetry.addData("Parking Spot", color);
            telemetry.addData("Red avg", averageRed);
            telemetry.addData("Green avg", averageGreen);
            telemetry.addData("Blue avg", averageBlue);
            telemetry.addData("Red 1", red1.get());
            telemetry.addData("Red 2", red2.get());
            telemetry.addData("Red 3", red3.get());
            telemetry.addData("Blue 1", blue1.get());
            telemetry.addData("Blue 2", blue2.get());
            telemetry.addData("Blue 3", blue3.get());
            telemetry.addData("Color", colors);
            telemetry.addData("Region", region);
            telemetry.addData("Mean", "%d %d", (int) mu.val[0], (int) mu.val[2]);
            telemetry.update();
            return input;
        }
    }
}