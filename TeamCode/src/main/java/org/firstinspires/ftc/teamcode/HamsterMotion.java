package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class HamsterMotion extends Thread {

    public static int WIGGLE_LEFT = 1060;
    public static int WIGGLE_RIGHT = 1200;

    public static int ROTATE_360 = 3800;

    private static double PF = 1.0;

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final Gamepad gamepad;
    private final Servo tail;
    private final Servo linear2;

    public enum Direction {
        FORWARD,
        RIGHT,
        BACKWARD,
        LEFT
    }

    public HamsterMotion(DcMotor leftFoot, DcMotor rightFoot, Servo tail, Gamepad gamepad, Servo linear) {
        this.frontLeftDrive = leftFoot;
        this.frontRightDrive = rightFoot;
        this.gamepad = gamepad;
        this.tail = tail;
        this.linear2 = linear;
    }

    double left = 0.58;
    double right = 0.3;

    @Override
    public void run() {
        tail.setPosition(left);

        long startTime = System.currentTimeMillis();

        while (!isInterrupted()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFootPower;
            double rightFootPower;

            // POV Mode uses left stick to go forward and strafe, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad.left_stick_y;
            double turn = -gamepad.left_stick_x;
            boolean wiggle = gamepad.a;
            boolean pull = gamepad.b;

            leftFootPower = Range.clip(drive - turn, -1.0, 1.0) * PF;
            rightFootPower = Range.clip(drive + turn, -1.0, 1.0) * PF;

            if (wiggle) {
                if(System.currentTimeMillis()-startTime > 200) {
                    if (tail.getPosition() >= left) {
                        startTime = System.currentTimeMillis();
                        tail.setPosition(right);
                    } else if (tail.getPosition() <= right) {
                        startTime = System.currentTimeMillis();
                        tail.setPosition(left);
                    }
                }
            }

            if(gamepad.y) {
                // move to 0 degrees
                linear2.setPosition(0.2);
            }
            if (gamepad.x) {
                // move to 180 degrees
                linear2.setPosition(0.7);
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(leftFootPower);
            frontRightDrive.setPower(rightFootPower);
        }
    }
}
