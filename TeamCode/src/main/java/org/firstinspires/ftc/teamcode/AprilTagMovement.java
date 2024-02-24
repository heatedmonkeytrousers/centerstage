package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "April Tag Movement", group = "Robot")
public class AprilTagMovement extends CameraSetupOpMode{
    public AprilTagMovement() {

    }
    @Override
    public void runOpMode() throws InterruptedException {
        set = false;
        pose = true;
        super.runOpMode();
    }
}
