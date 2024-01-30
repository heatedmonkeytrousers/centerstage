package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous: Near Board Red", group = "Robot")
public class AutonomousNearBoardRed extends AutonomousNearBoard {
    public AutonomousNearBoardRed() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.red = true;
        super.runOpMode();
    }
}