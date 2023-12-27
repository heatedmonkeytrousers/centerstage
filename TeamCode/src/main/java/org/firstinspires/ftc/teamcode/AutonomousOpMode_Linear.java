package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Robot: Generic Automation", group="Robot")

public class AutonomousOpMode_Linear extends StandardSetupOpMode {
    public AutonomousOpMode_Linear() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        // Initial parking spot
        Motion.PARKING_SPOT parkingSpot = position;

        // Reset the 30 second runtime timer
        runtime.reset();

        // Wait to start autonomous
        waitForStart();

        // Determine parking spot
        parkingSpot = position;
        telemetry.addData("Parking Spot", parkingSpot);
        telemetry.update();
    }
}