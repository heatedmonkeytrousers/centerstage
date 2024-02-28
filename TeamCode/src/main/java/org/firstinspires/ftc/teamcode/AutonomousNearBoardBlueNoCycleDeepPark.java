package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Near Board Blue No Cycle Deep Park", group = "Robot")
public class AutonomousNearBoardBlueNoCycleDeepPark extends AutonomousNearBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.cycle = false;
        super.deepPark = true;
        super.red = false;
        super.runOpMode();
    }
}