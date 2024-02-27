package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Near Board Red No Cycle", group = "Robot")
public class AutonomousNearBoardRedNoCycle extends AutonomousNearBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.cycle = false;
        super.red = true;
        super.runOpMode();
    }
}