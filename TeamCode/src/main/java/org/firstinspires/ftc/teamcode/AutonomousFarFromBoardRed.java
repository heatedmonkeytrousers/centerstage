package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Far From Board Red", group = "Robot")
public class AutonomousFarFromBoardRed extends AutonomousFarFromBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.red = true;
        super.runOpMode();
    }
}