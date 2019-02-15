package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group = "Learning")
@Disabled

public class ElapsedTimeTest extends LinearOpMode {

    public ElapsedTime mRunTime;

    public void setmRunTime() {
        this.mRunTime = new ElapsedTime();
    }

    public void runOpMode() {

        setmRunTime();

        waitForStart();
        this.mRunTime.reset();

        while(opModeIsActive()){

            telemetry.addData("Time since last reset: ", mRunTime.time());
            telemetry.update();
        }
    }
}
