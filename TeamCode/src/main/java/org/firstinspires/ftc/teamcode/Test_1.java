package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled

public class Test_1 extends LinearOpMode {

    private DcMotor armMotor;
    private CRServo collectorServo;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;
    private Servo armServo;
    private Servo armServo2;
    private Servo markerServo;

    @Override

    public void runOpMode() {

            armMotor = hardwareMap.get(DcMotor.class, "armMotor");
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            armServo2 = hardwareMap.get(Servo.class, "armServo2");
            armServo = hardwareMap.get(Servo.class, "armServo");
            markerServo = hardwareMap.get(Servo.class, "markerServo");
            collectorServo = hardwareMap.get(CRServo.class, "collectorServo");
            double power;
            double power2;
            double position = 0;
            int collectorPower = 0;

            //armServo.setPosition(0);
            //armServo2.setPosition(0);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            telemetry.addData("Status", "Initializare");
            telemetry.update(); // Wait for the game to start (driver presses PLAY)
            waitForStart();
            /* run until the end of the match (driver presses STOP) */
            while (opModeIsActive()) {

                if(armMotor.getCurrentPosition() > -4800 && gamepad2.left_stick_y < 0)
                    if(armMotor.getCurrentPosition() > -2100)
                        armMotor.setPower(gamepad2.left_stick_y/1.6);
                    else armMotor.setPower(gamepad2.left_stick_y/2.5);
                else if(armMotor.getCurrentPosition() < -20 && gamepad2.left_stick_y > 0)
                    if(armMotor.getCurrentPosition() < -2100)
                        armMotor.setPower(gamepad2.left_stick_y/1.6);
                    else armMotor.setPower(gamepad2.left_stick_y/2.5);
                else armMotor.setPower(0);

                telemetry.addData("Motor Encoder", armMotor.getCurrentPosition());
                telemetry.update();

                if(gamepad2.x && collectorPower != 0){
                    collectorPower=0;
                    sleep(500);
                }
                if (gamepad2.a && collectorPower != 1) {
                    collectorPower = 1;
                    sleep(500);
                }
                if (gamepad2.y && collectorPower != -1) {
                    collectorPower = -1;
                    sleep(500);
                }
                collectorServo.setPower(collectorPower);

                if (gamepad1.dpad_up){
                    markerServo.setPosition(markerServo.getPosition() + 0.02);
                    sleep(30);
                }
                else if(gamepad1.dpad_down){
                    markerServo.setPosition(markerServo.getPosition() - 0.02);
                    sleep(30);
                }

                //1-(-1/(armMotor.getCurrentPosition() + 2100))

                if (gamepad2.dpad_up && position < 1) {
                    armServo.setPosition(position + 0.02);
                    armServo2.setPosition(position + 0.02);
                    position += 0.02;
                    sleep(30);
                }
                if (gamepad2.dpad_down && position > 0) {
                    armServo.setPosition(position - 0.02);
                    armServo2.setPosition(position - 0.02);
                    position -= 0.02;
                    sleep(30);
                }

                power = -gamepad1.left_stick_y / 2;
                power2 = gamepad1.right_stick_x / 2;
                //backLeftMotor.setPower(power - power2);
                //backRightMotor.setPower(power - power2);
                if(power > 0) {
                    backLeftMotor.setPower(power - power2);
                    backRightMotor.setPower(power + power2);
                }
                else {
                    backLeftMotor.setPower(power + power2);
                    backRightMotor.setPower(power - power2);
                }
            }
        }
    }


