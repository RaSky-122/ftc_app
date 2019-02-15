package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OnlyWheelMotors", group = "Simple")

public class JustTheGoddamnArmMotorIGuess extends LinearOpMode {

    //private DcMotor armMotor;
    //private CRServo grabbyArmServo;
    private DcMotor armMotor;
    //private DcMotor armMotorHex2;
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;

    @Override
    public void runOpMode() {
        //armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //grabbyArmServo = hardwareMap.get(CRServo.class, "grabbyArmServo");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //armMotorHex2 = hardwareMap.get(DcMotor.class, "armMotorHex2");

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotorHex2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double power = 0.75;

        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.dpad_up) {
                power += 0.05;
                sleep(200);
            }
            else if(gamepad1.dpad_down) {
                power -= 0.05;
                sleep(200);
            }

            if(-gamepad1.left_stick_y > 0){
                frontRightMotor.setPower(power);
                frontLeftMotor.setPower(power);
                backRightMotor.setPower(power);
                backLeftMotor.setPower(power);
            }
            else if(-gamepad1.left_stick_y < 0){
                frontRightMotor.setPower(-power);
                frontLeftMotor.setPower(-power);
                backRightMotor.setPower(-power);
                backLeftMotor.setPower(-power);
            }
            else {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0) ;
            }

            telemetry.addData("Motor Power", power);
            telemetry.update();

            /**armMotor.setPower(gamepad1.left_stick_y);
            if(gamepad1.a)
                grabbyArmServo.setPower(1);
            else if(gamepad1.b)
                grabbyArmServo.setPower(-1);
            else grabbyArmServo.setPower(0);
             */
        }
    }
}
