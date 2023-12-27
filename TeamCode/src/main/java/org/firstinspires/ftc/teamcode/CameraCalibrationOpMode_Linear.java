package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Camera Calibration", group = "Linear Opmode")
public class CameraCalibrationOpMode_Linear extends CameraSetupOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Parking Spot", position);
            telemetry.addLine("Waiting for start");
            telemetry.addData("Red Total", (int) redTot);
            telemetry.addData("Blue Total", (int) blueTot);
            telemetry.addData("Mean", "%d %d", (int) mu.val[0], (int) mu.val[2]);
            telemetry.update();
            sleep(50);
        }
    }
}
