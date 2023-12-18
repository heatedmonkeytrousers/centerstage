package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
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

public class ColorCamera extends LinearOpMode{
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new ColorDetection());

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    public static class ColorDetection extends OpenCvPipeline
    {

        private int red;
        private int blue;
        //private int green;

        public enum PARKING_SPOT {
            PARK_ONE,
            PARK_TWO,
        }

        private volatile PARKING_SPOT position = PARKING_SPOT.PARK_ONE;


        @Override
        public Mat processFrame(Mat input)
        {
            Mat redOutput = new Mat();
            Mat blueOutput = new Mat();
            //Mat greenOutput = new Mat();

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */

            Imgproc.rectangle(
                    input,
                    new Point (
                            input.cols()/3,
                            input.rows()/2.5),
                    new Point(
                            input.cols()*(3f/6f),
                            input.rows()*(3f/5f)),
                    new Scalar(0, 0, 0), 1);

            /*
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */
            Core.extractChannel(input, redOutput, 0);
            //Core.extractChannel(input, greenOutput, 1);
            Core.extractChannel(input, blueOutput, 2);

            Imgproc.threshold(redOutput, redOutput, 150, 255, Imgproc.THRESH_BINARY);
            //Imgproc.threshold(greenOutput, greenOutput, 150, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(blueOutput, blueOutput, 150, 255, Imgproc.THRESH_BINARY);

            red = Core.countNonZero(redOutput);
            blue = Core.countNonZero(blueOutput);
            //green = Core.countNonZero(greenOutput);

            if (red > blue) {
                position = PARKING_SPOT.PARK_ONE;

            } else  {
                position = PARKING_SPOT.PARK_TWO;
            }

            return input;
        }

        public ColorCamera.ColorDetection.PARKING_SPOT getAnalysis()
        {
            return position;
        }
    }
}

