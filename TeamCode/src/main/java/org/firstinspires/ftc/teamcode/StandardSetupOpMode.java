package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Robot Setup Super Class", group = "Robot")
@Disabled
public class StandardSetupOpMode extends CameraSetupOpMode {

    // Declare Robot Setup members.
    protected ElapsedTime runtime = new ElapsedTime();
    protected DcMotor frontLeftDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor rearLeftDrive = null;
    protected DcMotor rearRightDrive = null;

    protected Motion motion = null;
    protected Arm arm = null;
    protected Shoulder shoulder = null;
    protected Claw claw = null;
    protected Wrist wrist = null;

    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;
    private DcMotor shoulderDrive = null;
    private Servo wristServo = null;
    private AnalogInput wristAnalog = null;
    private Servo clawServo1 = null;
    private Servo clawServo2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Call camera setup
        super.runOpMode();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive"); //ch3
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); //ch2
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeftDrive"); //ch1
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRightDrive"); //ch0
        armDrive1 = hardwareMap.get(DcMotor.class, "armDrive1"); //ch1 expansion hub Motor
        armDrive2 = hardwareMap.get(DcMotor.class, "armDrive2"); //ch2 expansion hub Motor
        shoulderDrive = hardwareMap.get(DcMotor.class, "shoulderDrive"); //ch0 Motor
        wristServo = hardwareMap.get(Servo.class, "wrist"); //ch0 expansion hub Servo
        wristAnalog = hardwareMap.get(AnalogInput.class, "wristAnalog"); //ch1 expansion hub analog input
        clawServo1 = hardwareMap.get(Servo.class, "clawServo1"); //ch1 expansion hub Servo
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2"); //ch2 expansion hub Servo


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        armDrive2.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        clawServo1.setDirection(Servo.Direction.FORWARD);
        clawServo2.setDirection(Servo.Direction.FORWARD);

        armDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulderDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        armDrive1.setTargetPosition(0);
        armDrive2.setTargetPosition(0);
        shoulderDrive.setTargetPosition(0);

        // Build the Motion class and give it a motion object
        motion = new Motion(frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive, null, null, null);
        arm = new Arm(armDrive1, armDrive2, null, null);
        shoulder = new Shoulder(shoulderDrive, arm, gamepad2);
        arm.setShoulder(shoulder);
        wrist = new Wrist(wristServo, wristAnalog, shoulder, null);
        claw = new Claw(gamepad2, clawServo1, clawServo2, null);
    }
}