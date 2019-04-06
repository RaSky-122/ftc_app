package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class asdfg extends LinearOpMode {

    ElapsedTime runTime;

    public void setRunTime() {
        runTime = new ElapsedTime();
    }

    public void runOpMode() {

        setRunTime();

        waitForStart();

        while(opModeIsActive()){

            runTime.startTime();
            telemetry.addData("runTime", runTime.milliseconds());
            telemetry.update();
        }
    }
}
