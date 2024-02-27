package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Far From Board Blue 5s Pause", group = "Robot")
public class AutonomousFarFromBoardBlue5sPause extends AutonomousFarFromBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.partnerWaitSeconds = 5.0;
        super.red = false;
        super.runOpMode();
    }
}