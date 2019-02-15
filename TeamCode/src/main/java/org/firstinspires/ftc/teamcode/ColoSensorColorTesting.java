package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "TestingColorSensor", group = "Learning")
@Disabled

public class ColoSensorColorTesting extends LinearOpMode {

    private ColorSensor yellowAndWhite;

    @Override

    public void runOpMode() {

        yellowAndWhite = hardwareMap.get(ColorSensor.class, "yellowAndWhite");
        yellowAndWhite.enableLed(true);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Red value: ", yellowAndWhite.red());
            telemetry.addData("Green value: ", yellowAndWhite.green());
            telemetry.addData("Blue value: ", yellowAndWhite.blue());
            telemetry.update();
        }
    }
}
