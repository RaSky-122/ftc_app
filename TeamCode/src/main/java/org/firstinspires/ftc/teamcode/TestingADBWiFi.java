package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled

public class TestingADBWiFi extends LinearOpMode {

    ElapsedTime timeStuff;
    public void setTimeStuff(){this.timeStuff = new ElapsedTime();}

    public void runOpMode(){

        setTimeStuff();

        waitForStart();

        timeStuff.reset();

        while(opModeIsActive()){

            telemetry.addData("Time: ", timeStuff.milliseconds());
            System.out.println("Time: " + timeStuff.milliseconds());
            telemetry.update();
            if(timeStuff.milliseconds() > 900)
                timeStuff.reset();
        }
    }
}
