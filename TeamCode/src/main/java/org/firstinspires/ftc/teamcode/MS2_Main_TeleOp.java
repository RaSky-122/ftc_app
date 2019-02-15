package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Main Driver Op", group = "Main")
@Disabled

public class MS2_Main_TeleOp extends LinearOpMode {

    /*private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    */
    private DcMotor armMotor;
    private DcMotor liftMotor;
    private CRServo armBaseServo;

    public void runOpMode(){

        /*frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");*/
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        armBaseServo = hardwareMap.get(CRServo.class, "armBaseServo");

        /*frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double power = 0.25;

        //MovingMotor direction = new MovingMotor();

        waitForStart();/**vezi liftMotor encoder*/

        while(opModeIsActive()){

            if(gamepad2.right_bumper && power <= 1) {
                power += 0.05;
                sleep(200);
            }
            else if(gamepad2.left_bumper && power >= -1) {
                power -= 0.05;
                sleep(200);
            }

            telemetry.addData("MotorPower ", power);


            int armPosition = armMotor.getCurrentPosition();
            int liftPosition = liftMotor.getCurrentPosition();
            telemetry.addData("Arm Motor Encoder: ", armPosition);
            telemetry.addData("Lift Motor Encoder: ", liftPosition);
            telemetry.update();

            if(gamepad2.dpad_up && liftPosition <= 12300)
                liftMotor.setPower(power);
            else if (gamepad2.dpad_down && liftPosition >= 0)
                liftMotor.setPower(-power);
            else if(gamepad1.dpad_up)
                liftMotor.setPower(power);
            else if(gamepad1.dpad_down)
                liftMotor.setPower(-power);
            else liftMotor.setPower(0);

            if(gamepad2.right_stick_x > 0 && armPosition > -4200)
                armMotor.setPower(-power);
            else if(gamepad2.right_stick_x < 0 && armPosition <= 0)
                armMotor.setPower(power);
            else if(gamepad1.right_stick_x > 0)
                armMotor.setPower(-power);
            else if(gamepad1.right_stick_x < 0)
                armMotor.setPower(power);
            else armMotor.setPower(0);

            if(gamepad2.x){
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(gamepad2.y){
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
/*
            if(gamepad1.dpad_up)
                direction.Forwards(power);
            else if(gamepad1.dpad_down)
                direction.Backwards(power);
            else if(gamepad1.dpad_right)
                direction.Right(power);
            else if(gamepad1.dpad_left)
                direction.Left(power);
            else if(gamepad1.left_stick_x < 0)
                direction.RotateLeft(power);
            else if(gamepad1.left_stick_x > 0)
                direction.RotateRight(power);
            else if(gamepad1.x)
                direction.UpLeft(power);
            else if(gamepad1.y)
                direction.UpRight(power);
            else if(gamepad1.a)
                direction.DownLeft(power);
            else if(gamepad1.b)
                direction.DownRight(power);
            else direction.Stop();
            */
        }
    }
/*
    class MovingMotor{
        public void Stop() {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
        public void Forwards(double power) {
            //gamepad1.dpad_up
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
        }
        public void Backwards(double power) {
            //gamepad1.dpad_down
            frontRightMotor.setPower(-power);
            frontLeftMotor.setPower(-power);
            backRightMotor.setPower(-power);
            backLeftMotor.setPower(-power);
        }
        public void Right(double power) {
            //gamepad1.dpad_right
            frontLeftMotor.setPower(power*2);
            backLeftMotor.setPower((-power)*2);
            frontRightMotor.setPower((-power)*2);
            backRightMotor.setPower(power*2);
        }
        public void Left(double power) {
            //gamepad1.dpad_left
            frontLeftMotor.setPower((-power)*2);
            backLeftMotor.setPower(power*2);
            frontRightMotor.setPower(power*2);
            backRightMotor.setPower((-power)*2);
        }
        public void RotateRight(double power) {
            //gamepad1.left_stick_x>0
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backRightMotor.setPower(-power);
        }
        public void RotateLeft(double power) {
            //gamepad1.left_stick_x<0
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
        }
        public void UpLeft(double power) {
            //gamepad1.y
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(power*2);
            frontRightMotor.setPower(power*2);
        }
        public void UpRight(double power) {
            //gamepad1.x
            frontLeftMotor.setPower(power*2);
            backRightMotor.setPower(power*2);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }
        public void DownRight(double power) {
            //gamepad1.b
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(-power*2);
            frontRightMotor.setPower(-power*2);
        }
        public void DownLeft(double power) {
            //gamepad1.a
            frontLeftMotor.setPower(-power*2);
            backRightMotor.setPower(-power*2);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }
    }
    */
}
