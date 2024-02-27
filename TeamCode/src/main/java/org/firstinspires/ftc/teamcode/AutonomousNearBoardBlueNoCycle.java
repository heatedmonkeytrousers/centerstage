package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutonomousNearBoard;

@Autonomous(name = "Autonomous: Near Board Blue No Cycle", group = "Robot")
public class AutonomousNearBoardBlueNoCycle extends AutonomousNearBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.cycle = false;
        super.red = false;
        super.runOpMode();
    }
}