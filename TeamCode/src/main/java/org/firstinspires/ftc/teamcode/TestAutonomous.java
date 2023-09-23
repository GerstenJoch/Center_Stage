package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Autonomous")
public class TestAutonomous extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive(this);

    @Override
    public void runOpMode() {
        drive.init();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive.Drive(5,5,.5);
        }
    }
}