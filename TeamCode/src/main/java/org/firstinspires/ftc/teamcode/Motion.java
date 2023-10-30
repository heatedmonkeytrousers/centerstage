package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

public class Motion extends Thread {
    public static int LOW_POSITION = 3941;
    public static int MIDDLE_POSITION = 6647;
    public static int HIGH_POSITION = 9700;

    public static double AUTO_SPEED = 0.6;

    public static int TRANSLATE_FB = 1060;
    public static int TRANSLATE_LR = 1200;

    public static int ROTATE_360 = 3800;

    private static double PF = 0.75;

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor rearLeftDrive;
    private final DcMotor rearRightDrive;
    private final Gamepad gamepad;

    public enum Direction {
        FORWARD,
        RIGHT,
        BACKWARD,
        LEFT
    }

    public enum PARKING_SPOT {
        PARK_ONE,
        PARK_TWO,
        PARK_THREE
    }

    public Motion(DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor rearLeftDrive, DcMotor rearRightDrive, Gamepad gamepad) {
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.rearLeftDrive = rearLeftDrive;
        this.rearRightDrive = rearRightDrive;
        this.gamepad = gamepad;
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            // Debug
            if(gamepad.start){
                rotation(Motion.Direction.RIGHT, 90, 0.5);
                continue;
            }
            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double rearLeftPower;
            double rearRightPower;

            // POV Mode uses left stick to go forward and strafe, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad.left_stick_y;
            double strafe = gamepad.left_stick_x;
            double turn = -gamepad.right_stick_x;

            frontLeftPower = Range.clip(drive + turn - strafe, -1.0, 1.0) * PF;
            rearLeftPower = Range.clip(drive + turn + strafe, -1.0, 1.0) * PF;
            frontRightPower = Range.clip(drive - turn + strafe, -1.0, 1.0) * PF;
            rearRightPower = Range.clip(drive - turn - strafe, -1.0, 1.0) * PF;

            if (gamepad.right_bumper) {
                frontLeftPower *= 1;
                rearLeftPower *= 1;
                frontRightPower *= 1;
                rearRightPower *= 1;

            } else if (gamepad.a) {
                rotation(Direction.LEFT, 180,1);
            } else if (gamepad.y) {
                rotation(Direction.RIGHT, 180,1);
            }
            else if (gamepad.b) {
                translate(Direction.RIGHT, 1.66, 1);
            } else if (gamepad.x) {
                translate(Direction.LEFT, 1.66, 1);
            } else if (gamepad.dpad_up) {
                translate(Direction.FORWARD, 1, 1);

            } else if (gamepad.dpad_down) {
                translate(Direction.BACKWARD, 1, 1);

            } else if (gamepad.dpad_left) {
                translate(Direction.LEFT, 1, 1);

            } else if (gamepad.dpad_right) {
                translate(Direction.RIGHT, 1, 1);
            } else if (gamepad.left_trigger > 0.5) {
                rotation(Direction.LEFT, 90, 1);

            } else if (gamepad.right_trigger > 0.5) {
                rotation(Direction.RIGHT, 90, 1);
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            rearLeftDrive.setPower(rearLeftPower);
            frontRightDrive.setPower(frontRightPower);
            rearRightDrive.setPower(rearRightPower);
        }
    }
    public void translate(Direction direction, double squares, double power) {
        translate(direction, squares ,power,false);
    }
    public void translate(Direction direction, double squares, double power, boolean auto) {

        //ensure good input
        power = Range.clip(Math.abs(power), 0, 1.0);

        // Get the currentModes
        DcMotor.RunMode frontLieftMode = frontLeftDrive.getMode();
        DcMotor.RunMode frontRightMode = frontRightDrive.getMode();
        DcMotor.RunMode rearRightMode = rearRightDrive.getMode();
        DcMotor.RunMode rearLeftMode = rearLeftDrive.getMode();

        // Get current positions
        int frontLeftPosition = frontLeftDrive.getCurrentPosition();
        int frontRightPosition = frontRightDrive.getCurrentPosition();
        int rearRightPosition = rearRightDrive.getCurrentPosition();
        int rearLeftPosition = rearLeftDrive.getCurrentPosition();

        // Determine power
        switch (direction) {
            case FORWARD:
                frontLeftPosition -= TRANSLATE_FB * squares;
                frontRightPosition -= TRANSLATE_FB * squares;
                rearLeftPosition -= TRANSLATE_FB * squares;
                rearRightPosition -= TRANSLATE_FB * squares;
                break;
            case RIGHT:
                frontLeftPosition -= TRANSLATE_LR * squares;
                frontRightPosition += TRANSLATE_LR * squares;
                rearLeftPosition += TRANSLATE_LR * squares;
                rearRightPosition -= TRANSLATE_LR * squares;
                break;
            case BACKWARD:
                frontLeftPosition += TRANSLATE_FB * squares;
                frontRightPosition += TRANSLATE_FB * squares;
                rearLeftPosition += TRANSLATE_FB * squares;
                rearRightPosition += TRANSLATE_FB * squares;
                break;
            case LEFT:
                frontLeftPosition += TRANSLATE_LR * squares;
                frontRightPosition -= TRANSLATE_LR * squares;
                rearLeftPosition -= TRANSLATE_LR * squares;
                rearRightPosition += TRANSLATE_LR * squares;
                break;
            default:
                // We should never get here!
                return;
        }

        // Move until new positions
        frontLeftDrive.setTargetPosition(frontLeftPosition);
        frontRightDrive.setTargetPosition(frontRightPosition);
        rearLeftDrive.setTargetPosition(rearLeftPosition);
        rearRightDrive.setTargetPosition(rearRightPosition);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // will wait till in position
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);


        while (true) {
            boolean frontDone = !frontLeftDrive.isBusy() && !frontRightDrive.isBusy();
            boolean rearDone = !rearLeftDrive.isBusy() && !rearRightDrive.isBusy();
            if (auto) {
                if (frontDone && rearDone) {
                    break;
                }
            } else {
                if (frontDone || rearDone) {
                    break;
                }
            }
        }

        // reset mode
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotation(Direction direction, double angle, double power) {
        rotation(direction, angle, power, false);
    }
    public void rotation(Direction direction, double angle, double power, boolean auto) {

        //ensure good input
        power = Range.clip(Math.abs(power), 0, 1.0);

        int rotation = (int) (ROTATE_360 * angle / 360);

        // Get the currentModes
        DcMotor.RunMode frontLeftMode = frontLeftDrive.getMode();
        DcMotor.RunMode frontRightMode = frontRightDrive.getMode();
        DcMotor.RunMode rearRightMode = rearRightDrive.getMode();
        DcMotor.RunMode rearLeftMode = rearLeftDrive.getMode();

        // Get current positions
        int frontLeftPosition = frontLeftDrive.getCurrentPosition();
        int frontRightPosition = frontRightDrive.getCurrentPosition();
        int rearRightPosition = rearRightDrive.getCurrentPosition();
        int rearLeftPosition = rearLeftDrive.getCurrentPosition();

        // Determine power
        switch (direction) {
            case FORWARD:
            case RIGHT:

                frontLeftPosition -= rotation;
                frontRightPosition += rotation;
                rearLeftPosition -= rotation;
                rearRightPosition += rotation;
                break;

            case BACKWARD:
            case LEFT:

                frontLeftPosition += rotation;
                frontRightPosition -= rotation;
                rearLeftPosition += rotation;
                rearRightPosition -= rotation;
                break;

            default:
                // We should never get here!
                return;
        }

        // Move until new positions
        frontLeftDrive.setTargetPosition(frontLeftPosition);
        frontRightDrive.setTargetPosition(frontRightPosition);
        rearLeftDrive.setTargetPosition(rearLeftPosition);
        rearRightDrive.setTargetPosition(rearRightPosition);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);

        // will wait till in position

        while (true) {
            boolean frontDone = !frontLeftDrive.isBusy() && !frontRightDrive.isBusy();
            boolean rearDone = !rearLeftDrive.isBusy() && !rearRightDrive.isBusy();
            if (auto) {
                if (frontDone && rearDone) {
                    break;
                }
            } else {
                if (frontDone || rearDone) {
                    break;
                }
            }
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        // reset mode
        frontLeftDrive.setMode(frontLeftMode);
        frontRightDrive.setMode(frontRightMode);
        rearLeftDrive.setMode(rearLeftMode);
        rearRightDrive.setMode(rearRightMode);


    }
}


