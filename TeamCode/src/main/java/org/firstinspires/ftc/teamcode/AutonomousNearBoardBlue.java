package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Near Board Blue", group = "Robot")
public class AutonomousNearBoardBlue extends AutonomousNearBoard {
    public AutonomousNearBoardBlue() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.red = false;
        super.runOpMode();
    }
}