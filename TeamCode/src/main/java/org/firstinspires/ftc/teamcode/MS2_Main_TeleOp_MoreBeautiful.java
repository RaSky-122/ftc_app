package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
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
    private DcMotor armBaseMotor;
    private Servo armBoxServo;
    private Servo armBoxServo2;
    private CRServo grabServo;
    private DcMotor collectorMotor;
    private BNO055IMU imu;

    double power = 0.2;
    double power2 = 0.75;

    double savedGyro = 0;

    public void runOpMode(){

        Initialize init = new Initialize();

        init.imuInit();

        init.motorsInit();

        ServoMovement servoStuff = new ServoMovement();
        Gamepad2Motors gamepad2Motors = new Gamepad2Motors();

        waitForStart();/**vezi liftMotor encoder*/

        while(opModeIsActive()){

            servoStuff.grabServoArm();
            gamepad2Motors.moveArmBaseMotor();
            gamepad2Motors.moveCollectorMotor();
            servoStuff.moveBoxServo();
            gamepad2Motors.moveLiftMotor();
            gamepad2Motors.moveArmMotor();
            smoothMovement();

            telemetry.addData("Arm Encoder ", armMotor.getCurrentPosition());
            telemetry.addData("Lift Encoder ", liftMotor.getCurrentPosition());
            telemetry.addData("Arm Base Encoder ", armBaseMotor.getCurrentPosition());
            telemetry.addData("Collector Motor Encoder ", collectorMotor.getCurrentPosition());
            telemetry.update();
        }

        armBoxServo2.close();
        armBoxServo.close();
        grabServo.close();
    }

    class ServoMovement {
        public void moveBoxServo() {
            if (gamepad2.b && Math.abs(liftMotor.getCurrentPosition()) >= 4000) {
                armBoxServo.setPosition(0);
                armBoxServo2.setPosition(0);
            } else if (Math.abs(liftMotor.getCurrentPosition()) >= 4000) {
                armBoxServo.setPosition(1);
                armBoxServo2.setPosition(1);
            }
        }

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

        frontLeftMotor.setPower((((-gamepad1.left_stick_y)+gamepad1.left_stick_x*2)/2)*power+gamepad1.right_stick_x/2);

        backRightMotor.setPower((((-gamepad1.left_stick_y)+gamepad1.left_stick_x*2)/2)*power-gamepad1.right_stick_x/2);

        frontRightMotor.setPower((((-gamepad1.left_stick_y)+(-gamepad1.left_stick_x*2))/2)*power-gamepad1.right_stick_x/2);

        backLeftMotor.setPower((((-gamepad1.left_stick_y)+(-gamepad1.left_stick_x*2))/2)*power+gamepad1.right_stick_x/2);

        boolean motorsOn = false;

        if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0)
            motorsOn = true;

        if(motorsOn && power < 0.8)
            power += 0.04;
        else if(motorsOn && power > 0.8)
            power = 0.8;
        else if(!motorsOn)
            power = 0.2;
    }

    class Gamepad2Motors{

        public  void moveLiftMotor(){
            int armPosition = armMotor.getCurrentPosition();
            int liftPosition = liftMotor.getCurrentPosition();

            if(-gamepad2.right_stick_y > 0 && (liftMotor.getCurrentPosition() < 5900 || gamepad2.left_bumper) && ((Math.abs(armPosition) >= 800 && liftPosition <= 2900) || Math.abs(armPosition) >= 1000))
                liftMotor.setPower(1);
            else if (-gamepad2.right_stick_y < 0 && ((liftPosition > 0 || gamepad2.left_bumper)))
                liftMotor.setPower(-1);
            else liftMotor.setPower(0);

            if(gamepad2.left_trigger > 0.8) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public  void moveArmMotor(){
            if(gamepad2.a)
                armMotor.setPower(-power2);
            else if(gamepad2.y)
                armMotor.setPower(power2);
            else armMotor.setPower(0);

            if(gamepad2.left_trigger > 0.8) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void moveArmBaseMotor(){
            if (gamepad2.dpad_right && (armBaseMotor.getCurrentPosition() < 940 || gamepad2.left_bumper)){
                armBaseMotor.setPower(1);
            }
            else if(gamepad2.dpad_left && (armBaseMotor.getCurrentPosition() > 10 || gamepad2.left_bumper)){
                armBaseMotor.setPower(-1);
            }
            else armBaseMotor.setPower(0);
        }

        public void moveCollectorMotor(){

            if(gamepad2.dpad_up) {
                collectorMotor.setPower(1);
            }
            else if(gamepad2.dpad_down) {
                collectorMotor.setPower(-1);
            }
            else {
                collectorMotor.setPower(0);
            }
        }
    }

    class Initialize{

        public void motorsInit(){
            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
            armMotor = hardwareMap.get(DcMotor.class, "armMotor");
            liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
            armBaseMotor = hardwareMap.get(DcMotor.class, "armBaseMotor");
            armBoxServo = hardwareMap.get(Servo.class, "armBoxServo");
            armBoxServo2 = hardwareMap.get(Servo.class, "armBoxServo2");
            grabServo = hardwareMap.get(CRServo.class, "grabServo");
            collectorMotor = hardwareMap.get(DcMotor.class, "collectorMotor");

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
            collectorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            collectorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armBaseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armBaseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armBoxServo2.setDirection(Servo.Direction.REVERSE);
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