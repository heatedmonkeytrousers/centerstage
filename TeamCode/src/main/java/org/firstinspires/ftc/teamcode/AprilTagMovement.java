package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "April Tag Movement", group = "Robot")
public class AprilTagMovement extends CameraSetupOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        set();
        DcMotor armDrive1 = hardwareMap.get(DcMotor.class, "armDrive1"); //ch1 expansion hub Motor
        DcMotor armDrive2 = hardwareMap.get(DcMotor.class, "armDrive2"); //ch2 expansion hub Motor
        DcMotor shoulderDrive = hardwareMap.get(DcMotor.class, "shoulderDrive"); //ch0 Motor
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
        aprilTagPose(BLUE_STACK_WALL, DROP_DISTANCE, LEFT_DISTANCE);
        sleep(30000);
    }
}
