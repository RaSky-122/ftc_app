package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.nio.channels.DatagramChannel;

@TeleOp(name = "Main Driver Op", group = "Main")

public class MS2_Main_TeleOp_MoreBeautiful extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private DcMotor liftMotor;
    private DcMotor armExtensionMotor;
    private CRServo grabServo;
    private BNO055IMU imu;

    double power = 0.5;
    double power2 = 0.75;
    int directie = 0;

    double savedGyro = 0;

    public void runOpMode(){

        Initialize init = new Initialize();

        init.imuInit();

        init.motorsInit();

        ServoMovement servoStuff = new ServoMovement();
        Gamepad2Motors gamepad2Motors = new Gamepad2Motors();

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Waiting for start ", "...");
            telemetry.update();
        }

        while(opModeIsActive()){

            servoStuff.grabServoArm();
            gamepad2Motors.moveLiftMotor();
            gamepad2Motors.moveArmMotor();
            gamepad2Motors.moveArmExtensionMotor();
            smoothMovement();


            telemetry.addData("Arm Encoder ", armMotor.getCurrentPosition());
            telemetry.addData("Lift Encoder ", liftMotor.getCurrentPosition());
            telemetry.addData("Extension Arm Encoder ", armExtensionMotor.getCurrentPosition());
            telemetry.addData("Gyro INTRINSIC ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));
            telemetry.addData("Gyro EXTRINSIC ", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));
            telemetry.update();
        }

        grabServo.close();
    }

    class ServoMovement {
        public void grabServoArm() {
            if (gamepad2.right_trigger > 0)
                grabServo.setPower(1);
            else if (gamepad2.right_bumper)
                grabServo.setPower(-1);
            else if (gamepad2.x)
                grabServo.setPower(0);
        }
    }


    public void smoothMovement(){
        if (directie==0) {
            if (gamepad1.b) {

                directie = 1;
                sleep(500);
            }
        }
        if(directie==1) {
            if (gamepad1.b) {
                directie = 0;
                sleep(500);
            }
        }
        telemetry.addData("Directie", directie);
        if(gamepad1.right_bumper){
            power=0.25;
        }
        else {
            power=0.5;
        }
        if(directie==0){
            if(gamepad1.dpad_up){
                frontLeftMotor.setPower(power+gamepad1.right_stick_x/2);
                frontRightMotor.setPower(power-gamepad1.right_stick_x/2);
                backRightMotor.setPower(power-gamepad1.right_stick_x/2);
                backLeftMotor.setPower(power+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_down){
                frontLeftMotor.setPower(-power+gamepad1.right_stick_x/2);
                frontRightMotor.setPower(-power-gamepad1.right_stick_x/2);
                backRightMotor.setPower(-power-gamepad1.right_stick_x/2);
                backLeftMotor.setPower(-power+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_right){
                frontLeftMotor.setPower(power+gamepad1.right_stick_x/2);
                frontRightMotor.setPower(-power-gamepad1.right_stick_x/2);
                backRightMotor.setPower(power-gamepad1.right_stick_x/2);
                backLeftMotor.setPower(-power+gamepad1.right_stick_x/2);
            }else
            if(gamepad1.dpad_left){
                frontLeftMotor.setPower(-power+gamepad1.right_stick_x/2);
                frontRightMotor.setPower(power-gamepad1.right_stick_x/2);
                backRightMotor.setPower(-power-gamepad1.right_stick_x/2);
                backLeftMotor.setPower(power+gamepad1.right_stick_x/2);
            }else if(!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right){
                frontLeftMotor.setPower(gamepad1.right_stick_x/2);
                frontRightMotor.setPower(-gamepad1.right_stick_x/2);
                backRightMotor.setPower(-gamepad1.right_stick_x/2);
                backLeftMotor.setPower(gamepad1.right_stick_x/2);
            }
        }
        else if (directie==1){
            if(gamepad1.dpad_left){
                frontLeftMotor.setPower(power+gamepad1.right_stick_x/4);
                frontRightMotor.setPower(power-gamepad1.right_stick_x/4);
                backRightMotor.setPower(power-gamepad1.right_stick_x/4);
                backLeftMotor.setPower(power+gamepad1.right_stick_x/4);
            }else
            if(gamepad1.dpad_right){
                frontLeftMotor.setPower(-power+gamepad1.right_stick_x/4);
                frontRightMotor.setPower(-power-gamepad1.right_stick_x/4);
                backRightMotor.setPower(-power-gamepad1.right_stick_x/4);
                backLeftMotor.setPower(-power+gamepad1.right_stick_x/4);
            }else
            if(gamepad1.dpad_up){
                frontLeftMotor.setPower(power+gamepad1.right_stick_x/4);
                frontRightMotor.setPower(-power-gamepad1.right_stick_x/4);
                backRightMotor.setPower(power-gamepad1.right_stick_x/4);
                backLeftMotor.setPower(-power+gamepad1.right_stick_x/4);
            }else
            if(gamepad1.dpad_down){
                frontLeftMotor.setPower(-power+gamepad1.right_stick_x/4);
                frontRightMotor.setPower(power-gamepad1.right_stick_x/4);
                backRightMotor.setPower(-power-gamepad1.right_stick_x/4);
                backLeftMotor.setPower(power+gamepad1.right_stick_x/4);
            }else if(!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right){
                frontLeftMotor.setPower(gamepad1.right_stick_x/4);
                frontRightMotor.setPower(-gamepad1.right_stick_x/4);
                backRightMotor.setPower(-gamepad1.right_stick_x/4);
                backLeftMotor.setPower(gamepad1.right_stick_x/4);
            }
        }


    }

    class Gamepad2Motors{

        public  void moveLiftMotor(){
            int liftPosition = liftMotor.getCurrentPosition();

            if((((liftPosition < 5900 && -gamepad2.right_stick_y > 0) || liftPosition > 200 && -gamepad2.right_stick_y < 0))  || gamepad2.left_bumper)
                liftMotor.setPower(-gamepad2.right_stick_y);
            else liftMotor.setPower(0);

            if(gamepad2.left_trigger > 0.8) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public  void moveArmMotor(){
            if(gamepad2.a && (armMotor.getCurrentPosition() > 50 || gamepad2.left_bumper))
                armMotor.setPower(-power2);
            else if(gamepad2.y && (armMotor.getCurrentPosition() < 5200 || gamepad2.left_bumper))
                armMotor.setPower(power2);
            else armMotor.setPower(0);

            if(gamepad2.left_trigger > 0.8) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void moveArmExtensionMotor(){
            if(gamepad2.dpad_up && (armExtensionMotor.getCurrentPosition() < 5100 || gamepad2.left_bumper))
                armExtensionMotor.setPower(1);
            else if(gamepad2.dpad_down && (armExtensionMotor.getCurrentPosition() > 50 || gamepad2.left_bumper))
                armExtensionMotor.setPower(-1);
            else armExtensionMotor.setPower(0);

            if(gamepad2.left_trigger > 0.8){
                armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    class Initialize{

        public void motorsInit(){
            frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
            frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
            backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
            backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
            armMotor = hardwareMap.dcMotor.get("armMotor");
            armExtensionMotor = hardwareMap.dcMotor.get("armExtensionMotor");
            liftMotor = hardwareMap.dcMotor.get("liftMotor");
            grabServo = hardwareMap.crservo.get("grabServo");


            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void imuInit(){
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            BNO055IMU.Parameters params = new BNO055IMU.Parameters();

            params.mode                = BNO055IMU.SensorMode.IMU;
            params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            params.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            params.loggingEnabled      = true;
            params.loggingTag          = "IMU";
            params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(params);

            imu.startAccelerationIntegration(new Position(), new Velocity(), 500);
        }

    }

}











