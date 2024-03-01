package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto: Far From Board Red No Stack", group = "Robot")
public class AutonomousFarFromBoardRedNoStack extends AutonomousFarFromBoard {
    @Override
    public void runOpMode() throws InterruptedException {
        super.red = true;
        super.oldOverride = true;
        super.runOpMode();
    }
}