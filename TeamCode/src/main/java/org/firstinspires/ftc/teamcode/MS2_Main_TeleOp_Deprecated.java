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

@TeleOp(name = "Deprecated Driver Op", group = "Main")
@Disabled

public class MS2_Main_TeleOp_Deprecated extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private DcMotor liftMotor;
    private DcMotor armBaseMotor;
    private Servo armBoxServo;
    private Servo armBoxServo2;
    private Servo markerServo;
    private CRServo grabServo;
    private DcMotor collectorMotor;
    private BNO055IMU imu;

    double power = 0.2;
    double power2 = 0.75;

    double distance = 0;
    double auxDistance;

    double extensionTime = 0;

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
            telemetry.update();
        }

        armBoxServo2.close();
        armBoxServo.close();
        grabServo.close();
        markerServo.close();
    }

    public void changePower(){
        if((gamepad2.right_trigger > 0 || gamepad1.right_trigger > 0)&& power <= 1) {
            power += 0.05;
            sleep(200);
        }
        else if((gamepad2.left_trigger > 0 || gamepad1.left_trigger > 0)&& power >= -1) {
            power -= 0.05;
            sleep(200);
        }
        //telemetry.addData("Motor power ", power);
    }

    public void changePower2(){
        if(gamepad1.right_bumper)
            power = 0.2;
        else
            power = 0.5;

    }

    class ServoMovement {
        public void moveBoxServo(){
            if(gamepad2.b && Math.abs(liftMotor.getCurrentPosition()) >= 4000) {
                armBoxServo.setPosition(0);
                armBoxServo2.setPosition(0);
            }
            else if (Math.abs(liftMotor.getCurrentPosition()) >= 4000){
                armBoxServo.setPosition(1);
                armBoxServo2.setPosition(1);
            }
        }

        public void grabServoArm(){
            if(gamepad2.right_trigger > 0)
                grabServo.setPower(1);
            else if(gamepad2.right_bumper)
                grabServo.setPower(-1);
            else if(gamepad2.x)
                grabServo.setPower(0);
        }

        public void moveMarkerServo() {
            if(gamepad2.dpad_right)
                markerServo.setPosition(1);
            else markerServo.setPosition(0.2);
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

        if(motorsOn && power < 0.5)
            power += 0.04;
        else if(motorsOn && power > 0.5)
            power = 0.5;
        else if(!motorsOn)
            power = 0.2;
    }

    public void movingMotors(){

        MovingMotor direction = new MovingMotor();
        boolean motorsOn = false;


        if(-gamepad1.left_stick_y > 0) {
            if (gamepad1.left_stick_x < 0)
                motorsOn = direction.UpLeft(power);
            else if (gamepad1.left_stick_x > 0)
                motorsOn = direction.UpRight(power);
            else motorsOn = direction.Forwards(power);
        }
        else if(-gamepad1.right_stick_y > 0)
            motorsOn = direction.Forwards(power);

        if(-gamepad1.left_stick_y < 0) {
            if (gamepad1.left_stick_x < 0)
                motorsOn = direction.DownLeft(power);
            else if (gamepad1.left_stick_x > 0)
                motorsOn = direction.DownRight(power);
            else motorsOn = direction.Forwards(power);
        }
        else if(-gamepad1.right_stick_y < 0)
            motorsOn = direction.Backwards(power);

        else if(gamepad1.left_stick_x > 0)
            motorsOn = direction.Right(power);
        else if(gamepad1.left_stick_x < 0)
            motorsOn = direction.Left(power);
        else if(gamepad1.right_stick_x < 0)

            motorsOn = direction.RotateLeft(power);
        else if(gamepad1.right_stick_x > 0)
            motorsOn = direction.RotateRight(power);

        else motorsOn = direction.Stop();
    }

    public void moveArms(){
        int armPosition = armMotor.getCurrentPosition();
        int liftPosition = liftMotor.getCurrentPosition();

        if(-gamepad2.right_stick_y > 0 && ((liftPosition <= 5000 || gamepad2.left_bumper) && ((Math.abs(armPosition) >= 800 && liftPosition <= 2900) || Math.abs(armPosition) >= 1000)))
            liftMotor.setPower(1);
        else if (-gamepad2.right_stick_y < 0 && ((liftPosition >= 0 || gamepad2.left_bumper)))
            liftMotor.setPower(-1);
        else liftMotor.setPower(0);

        if(gamepad2.y)
            armMotor.setPower(-power2);
        else if(gamepad2.a)
            armMotor.setPower(power2);
        else armMotor.setPower(0);

        if(gamepad2.left_trigger > 0.8) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    class Gamepad2Motors{

        public  void moveLiftMotor(){
            int armPosition = armMotor.getCurrentPosition();
            int liftPosition = liftMotor.getCurrentPosition();

            if(-gamepad2.right_stick_y > 0 && ((liftPosition <= 5000 || gamepad2.left_bumper) && ((Math.abs(armPosition) >= 800 && liftPosition <= 2900) || Math.abs(armPosition) >= 1000)))
                liftMotor.setPower(1);
            else if (-gamepad2.right_stick_y < 0 && ((liftPosition >= 0 || gamepad2.left_bumper)))
                liftMotor.setPower(-1);
            else liftMotor.setPower(0);

            if(gamepad2.left_trigger > 0.8) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public  void moveArmMotor(){
            if(gamepad2.y)
                armMotor.setPower(-power2);
            else if(gamepad2.a)
                armMotor.setPower(power2);
            else armMotor.setPower(0);

            if(gamepad2.left_trigger > 0.8) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public void moveArmBaseMotor(){
            if (gamepad2.dpad_up && armBaseMotor.getCurrentPosition() < 940){
                armBaseMotor.setPower(1);
            }
            else if(gamepad2.dpad_down && armBaseMotor.getCurrentPosition() > 10){
                armBaseMotor.setPower(-1);
            }
            else armBaseMotor.setPower(0);
        }

        public void moveCollectorMotor(){

            if(gamepad2.dpad_left)
                collectorMotor.setPower(0.7);
            else if(gamepad2.dpad_right)
                collectorMotor.setPower(-0.7);
            else collectorMotor.setPower(0);
        }
    }

    class MovingMotor{
        public boolean Stop() {
            frontLeftMotor.setPower(frontLeftMotor.getPower() - 0.02);
            frontRightMotor.setPower(frontRightMotor.getPower() - 0.02);
            backLeftMotor.setPower(backLeftMotor.getPower() - 0.02);
            backRightMotor.setPower(backRightMotor.getPower() - 0.02);
            /*if(power == 0.2){
                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }*/
            return false;
        }
        public boolean Forwards(double power) {
            //gamepad1.dpad_up
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
            return true;
        }
        public boolean Backwards(double power) {
            //gamepad1.dpad_down
            frontRightMotor.setPower(-power);
            frontLeftMotor.setPower(-power);
            backRightMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            return true;
        }
        public boolean Right(double power) {
            //gamepad1.dpad_right
            frontLeftMotor.setPower(power*1.5);
            backLeftMotor.setPower((-power)*1.5);
            frontRightMotor.setPower((-power)*1.5);
            backRightMotor.setPower(power*1.5);
            return true;
        }
        public boolean Left(double power) {
            //gamepad1.dpad_left
            frontLeftMotor.setPower((-power)*1.5);
            backLeftMotor.setPower(power*1.5);
            frontRightMotor.setPower(power*1.5);
            backRightMotor.setPower((-power)*1.5);
            return true;
        }
        public boolean RotateRight(double power) {
            //gamepad1.left_stick_x>0
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backRightMotor.setPower(-power);
            return true;
        }
        public boolean RotateLeft(double power) {
            //gamepad1.left_stick_x<0
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            return true;
        }
        public boolean UpLeft(double power) {
            //gamepad1.y
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(power*2);
            frontRightMotor.setPower(power*2);
            return true;
        }
        public boolean UpRight(double power) {
            //gamepad1.x
            frontLeftMotor.setPower(power*2);
            backRightMotor.setPower(power*2);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            return true;
        }
        public boolean DownRight(double power) {
            //gamepad1.b
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(-power*2);
            frontRightMotor.setPower(-power*2);
            return true;
        }
        public boolean DownLeft(double power) {
            //gamepad1.a
            frontLeftMotor.setPower(-power*2);
            backRightMotor.setPower(-power*2);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            return true;
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
            markerServo = hardwareMap.get(Servo.class, "markerServo");
            grabServo = hardwareMap.get(CRServo.class, "grabServo");
            collectorMotor = hardwareMap.get(DcMotor.class, "collectorMotor");

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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