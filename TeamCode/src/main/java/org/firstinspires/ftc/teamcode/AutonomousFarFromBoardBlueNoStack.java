package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Far From Board Blue No Stack", group = "Robot")
public class AutonomousFarFromBoardBlueNoStack extends AutonomousFarFromBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.red = false;
        super.oldOverride = true;
        super.runOpMode();
    }
}