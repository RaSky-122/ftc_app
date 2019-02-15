package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Just 1 Motor", group = "Test")

public class Just1Mo extends LinearOpMode {

    private DcMotor someMotor;
    double power = 0.3;

    public void runOpMode(){

        someMotor = hardwareMap.get(DcMotor.class, "armBaseMotor");
        someMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        someMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        someMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.right_bumper && power < 1) {
                power += 0.05;
                sleep(200);
            }
            else if(gamepad1.left_bumper && power > 0) {
                power -= 0.05;
                sleep(200);
            }

            if(gamepad1.left_trigger > 0.5){
                someMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                someMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(gamepad1.x){
                someMotor.setPower(power);
            }
            else if(gamepad1.b){
                someMotor.setPower(-power);
            }
            else someMotor.setPower(0);

            telemetry.addData("Power", power);
            telemetry.addData("Encoder", someMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
