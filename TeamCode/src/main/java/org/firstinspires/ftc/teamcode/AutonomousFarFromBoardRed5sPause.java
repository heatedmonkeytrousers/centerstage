package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Far From Board Red 5s Pause", group = "Robot")
public class AutonomousFarFromBoardRed5sPause extends AutonomousFarFromBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.partnerWaitSeconds = 5.0;
        super.red = true;
        super.runOpMode();
    }
}