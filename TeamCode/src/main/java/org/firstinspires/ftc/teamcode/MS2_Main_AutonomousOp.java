package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Crate", group = "Main")
@Disabled

public class MS2_Main_AutonomousOp extends LinearOpMode {

    private BNO055IMU imu;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor liftMotor;
    private DcMotor armMotor;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    VuforiaLocalizer vuforia;

    TFObjectDetector tfod;

    ElapsedTime timeStuff;

    public void setTimeStuff(){
        this.timeStuff = new ElapsedTime();
    }

    double power = 0.2;

    double initialGyro;
    double currentGyro;

    double initialEncoder;

    int goldPositionLeft;
    int goldPositionTop;

    int rotations = 0;

    boolean cubeFound = false;

    @Override
    public void runOpMode() {

        motorsInit();

        vuforiaInit();

        imuInit();

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackable bluePerimeter = beacons.get(0);
        bluePerimeter.setName("BluePerimeter");
        VuforiaTrackable redPerimeter = beacons.get(1);
        redPerimeter.setName("RedPerimeter");
        VuforiaTrackable frontPerimeter = beacons.get(2);
        frontPerimeter.setName("FrontPerimeter");
        VuforiaTrackable backPerimeter = beacons.get(3);
        backPerimeter.setName("BackPerimeter");

        VuforiaTrackableDefaultListener blue = new VuforiaTrackableDefaultListener();
        VuforiaTrackableDefaultListener red = new VuforiaTrackableDefaultListener();
        VuforiaTrackableDefaultListener back = new VuforiaTrackableDefaultListener();
        VuforiaTrackableDefaultListener front = new VuforiaTrackableDefaultListener();

        bluePerimeter.setListener(blue);
        redPerimeter.setListener(red);
        backPerimeter.setListener(back);
        frontPerimeter.setListener(front);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tfodInit();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        if (tfod != null) {
            tfod.activate();
        }


        //beacons.activate();
        /**********************************************************************************************************************/
        waitForStart();

        setTimeStuff();
        MovingMotor motorMovement = new MovingMotor();
        FindingMinerals findingMinerals = new FindingMinerals();

        if(opModeIsActive()){

            while(Math.abs(armMotor.getCurrentPosition()) <= 1300)
                armMotor.setPower(-0.3);
            armMotor.setPower(0);

            while(Math.abs(liftMotor.getCurrentPosition()) <= 5000 && opModeIsActive()) {
                liftMotor.setPower(1);
                telemetry.addData("Lift Encoder ", liftMotor.getCurrentPosition());
                telemetry.addData("Arm Encoder ", armMotor.getCurrentPosition());
                telemetry.update();
            }

            armMotor.setPower(0);
            liftMotor.setPower(0);

            findingMinerals.rotation(5, "right", 0.3,false);

            motorMovement.forwards(0.3, 500);

            findingMinerals.rotation(26, "right", 0.4,false);

            while(!cubeFound){
                findingMinerals.rotation(80, "left", 0.2,true);
                if(!cubeFound)
                    findingMinerals.rotation(80, "right", 0.2,true);
            }

            if(cubeFound){
                motorMovement.forwards(0.5, 2000);
            }

            motorMovement.stop();

            while(Math.abs(armMotor.getCurrentPosition()) <= 3300 && opModeIsActive()){
                armMotor.setPower(-0.3);
            }
        }
    }

    private class FindingMinerals{
        MovingMotor movingMotor = new MovingMotor();

        private void rotation(double targetAngle, String rotationDirection, double rPower,boolean search){
            resetGyro();
            while(Math.abs(currentGyro - initialGyro) <= targetAngle && opModeIsActive() && (!search || !cubeFound)){
                if(rotationDirection.equals("left"))
                    movingMotor.rotateLeft(rPower);
                else if(rotationDirection.equals("right"))
                    movingMotor.rotateRight(rPower);
                else movingMotor.stop();
                currentGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
                if(search)
                    scanGold(tfod.getUpdatedRecognitions());
            }
        }

        private void searchingForGold(){
            timeStuff.reset();
            while(timeStuff.milliseconds() <= 2000 && !scanGold(tfod.getUpdatedRecognitions()) && opModeIsActive() && !cubeFound){
                movingMotor.stop();
            }
        }
    }

    private void resetGyro(){
        initialGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        currentGyro = initialGyro;
    }

    private void tfodInit() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void vuforiaInit(){
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ARDQXkP/////AAABmZbc2Ud2I0b9miNIUDsTMNODX/myN9Y/VqGh0NCsRXeaus19hlL8b//KXthG4HyAyRKxJ2aKBRZ9A000NWP2u4val3zdjTlCeIa3k+xttxFnkPSz3WOe7zhygG6z4FQEvZe3a7H2MNk2f2hUGqibfceIIhoiPzMaMq+bue7z+n0lKs0TDOVofWDlYd8OWqB+PvTegu3imXsgIgAeIEb25fTfN9ke4mD37TUOLunYiDhSNrpk4u8gYikyE5SqkZOSGuI7ifFK09okk85Yhh1C+/FtWkUXD3qZwmLaKNJUeodiNzwekVrXbNtFBFWmhGq3GnF8+RpMH6d2qsX5BdvkFusWFDmfwG/9RaZOmN+CXISP";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.TEAPOT;

        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 7);
    }

    private void motorsInit(){
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void imuInit(){

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();

        params.angleUnit                        = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled                   = true;
        params.loggingTag                       = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(params);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
    }

    boolean scanGold(List<Recognition> updatedRecognitions){
        if(updatedRecognitions != null){
            for(Recognition recognition : updatedRecognitions){
                if(recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getTop() > 600 && recognition.getLeft() > 400) {
                    goldPositionLeft = (int) recognition.getLeft();
                    goldPositionTop = (int) recognition.getTop();
                    if(!cubeFound)
                        cubeFound = true;
                    return true;
                }
            }
        }
        return false;
    }

    public class MovingMotor{
        public void stop() {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
        public void forwards(double power, int targetPosition) {
            //gamepad1.dpad_up
            int initialEncoder = frontRightMotor.getCurrentPosition();
            if(initialEncoder != 0)
                while(opModeIsActive() && (Math.abs(frontRightMotor.getCurrentPosition() - initialEncoder) <= targetPosition)) {
                    frontLeftMotor.setPower(power);
                    frontRightMotor.setPower(power);
                    backLeftMotor.setPower(power);
                    backRightMotor.setPower(power);
            }
            else{
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(power);
                backLeftMotor.setPower(power);
                backRightMotor.setPower(power);
            }

        }
        public void backwards(double power, int targetPosition) {
            //gamepad1.dpad_down
            int initialEncoder = frontRightMotor.getCurrentPosition();
            if(initialEncoder != 0)
                while(opModeIsActive() && (Math.abs(frontRightMotor.getCurrentPosition() - initialEncoder) <= targetPosition)) {
                    frontLeftMotor.setPower(-power);
                    frontRightMotor.setPower(-power);
                    backLeftMotor.setPower(-power);
                    backRightMotor.setPower(-power);
                }
            else{
                frontLeftMotor.setPower(-power);
                frontRightMotor.setPower(-power);
                backLeftMotor.setPower(-power);
                backRightMotor.setPower(-power);
            }
        }
        public void rotateRight(double power) {
            //gamepad1.left_stick_x>0
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backRightMotor.setPower(-power);
        }
        public void rotateLeft(double power) {
            //gamepad1.left_stick_x<0
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
        }
    }
}
