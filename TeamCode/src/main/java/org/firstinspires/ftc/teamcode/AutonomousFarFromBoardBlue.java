package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Far From Board Blue", group = "Robot")
public class AutonomousFarFromBoardBlue extends AutonomousFarFromBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.red = false;
        super.runOpMode();
    }
}