package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Far From Board Blue", group = "Robot")
public class AutonomousFarFromBoardBlue extends AutonomousFarFromBoard {
    public AutonomousFarFromBoardBlue() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.red = false;
        super.runOpMode();
    }
}