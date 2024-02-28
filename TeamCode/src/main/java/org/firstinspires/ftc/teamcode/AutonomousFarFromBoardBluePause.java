package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Far From Board Blue Pause", group = "Robot")
public class AutonomousFarFromBoardBluePause extends AutonomousFarFromBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.partnerWaitSeconds = 2.0;
        super.red = false;
        super.runOpMode();
    }
}