package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "April Tag Movement", group = "Robot")
public class AprilTagMovement extends CameraSetupOpMode{
    private DcMotor shoulderDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;

    public AprilTagMovement() {

    }
    @Override
    public void runOpMode() throws InterruptedException {
        set = true;
        pose = true;
        armDrive1 = hardwareMap.get(DcMotor.class, "armDrive1"); //ch1 expansion hub Motor
        armDrive2 = hardwareMap.get(DcMotor.class, "armDrive2"); //ch2 expansion hub Motor
        shoulderDrive = hardwareMap.get(DcMotor.class, "shoulderDrive"); //ch0 Motor
        armDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        armDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        armDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive1.setTargetPosition(0);
        armDrive2.setTargetPosition(0);
        shoulderDrive.setTargetPosition(0);
        waitForStart();
        Arm arm = new Arm(armDrive1, armDrive2, gamepad2, null);
        Shoulder shoulder = new Shoulder(shoulderDrive, arm, gamepad2);
        shoulder.setShoulderPosition(0.7, -50);
        super.runOpMode();
        sleep(50000);
    }
}
