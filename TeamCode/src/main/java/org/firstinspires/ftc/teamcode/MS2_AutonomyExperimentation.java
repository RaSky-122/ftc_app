package org.firstinspires.ftc.teamcode;

import android.test.suitebuilder.annotation.MediumTest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
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

@Autonomous(name = "Autonomy Testing", group = "Learning")

public class MS2_AutonomyExperimentation extends LinearOpMode {

    private BNO055IMU imu;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor liftMotor;
    private DcMotor armMotor;
    private Servo markerServo;

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
    double originGyro;

    double initialEncoder;

    int silver1Position;
    int silver2Position;
    int goldPosition;

    int rotations = 0;

    boolean silver1Found = false;
    boolean silver2Found = false;
    boolean cubeFound = false;

    int elements = 0;

    @Override
    public void runOpMode() {

        Initialize initialize = new Initialize();

        initialize.motorsInit();

        initialize.vuforiaInit();

        initialize.imuInit();

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
            initialize.tfodInit();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        if (tfod != null) {
            tfod.activate();
        }


        //beacons.activate();
        /**********************************************************************************************************************/
        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Waiting for ", "start.");
            telemetry.update();
        }

        telemetry.addData("Starting ", "hopefully.");
        telemetry.update();

        setTimeStuff();
        MovingMotor motorMovement = new MovingMotor();
        FindingMinerals findingMinerals = new FindingMinerals();
        WheelEncoder wheelEncoder = new WheelEncoder();

        wheelEncoder.stop();

        if(opModeIsActive()){

            String cubeLocation = "not_found";

            while(opModeIsActive() && cubeLocation.equals("not_found")){
                cubeLocation = goldLocation();
            }

            while(Math.abs(liftMotor.getCurrentPosition()) <= 5900 && opModeIsActive()) {
                liftMotor.setPower(1);
            }

            liftMotor.setPower(0);

            findingMinerals.rotateFor(5, "right", 0.3);

            motorMovement.forwards(0.3, 300);

            switch (cubeLocation){
                case "right": findingMinerals.rotateTo(-28, 0.3);break;
                case "center": findingMinerals.rotateTo(-7, 0.3);break;
                case "left": findingMinerals.rotateTo(27, 0.3);break;
            }

            if(cubeLocation.equals("center")) {
                motorMovement.forwards(0.3,  950);
                motorMovement.backwards(0.5, 400);
            }
            else if(cubeLocation.equals("left")) {
                motorMovement.forwards(0.5, 1170);
                motorMovement.backwards(0.5, 500);
            }
            else {
                motorMovement.forwards(0.5, 1170);
                motorMovement.backwards(0.5, 500);
            }

            findingMinerals.rotateTo(77, 0.3);

            motorMovement.forwards(0.5, 2300);

            motorMovement.stop();

            tfod.shutdown();
        }
    }

    private class FindingMinerals{
        MovingMotor movingMotor = new MovingMotor();

        private void rotateFor(double targetAngle, String rotationDirection, double rPower){
            resetGyro();
            while(Math.abs(currentGyro - initialGyro) <= targetAngle && opModeIsActive()){
                if(rotationDirection.equals("left"))
                    movingMotor.rotateLeft(rPower);
                else if(rotationDirection.equals("right"))
                    movingMotor.rotateRight(rPower);
                else movingMotor.stop();
                currentGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
            }
            movingMotor.stop();
        }

        private void rotateTo(double targetAngle, double rPower) {

            currentGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

            if (currentGyro < targetAngle) {
                while (currentGyro < targetAngle && opModeIsActive()) {
                    currentGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
                    movingMotor.rotateLeft(rPower);
                }
            } else if (currentGyro > targetAngle) {
                while (currentGyro > targetAngle && opModeIsActive()) {
                    currentGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
                    movingMotor.rotateRight(rPower);
                }
            }
            movingMotor.stop();
        }
    }

    private void resetGyro(){
        initialGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
        currentGyro = initialGyro;
    }

    private class WheelEncoder{

        private void stop(){
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        private void start(){
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    int scanGold(List<Recognition> updatedRecognitions){
        int i = 0;
        if(updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && !cubeFound && recognition.getTop() > 350) {
                    goldPosition = (int) recognition.getLeft();
                    cubeFound = true;
                    i++;
                    }
                else if(recognition.getLabel().equals(LABEL_SILVER_MINERAL) && !silver1Found && recognition.getTop() > 350) {
                    silver1Position = (int) recognition.getLeft();
                    silver1Found = true;
                    i++;
                }
                else if(recognition.getLabel().equals(LABEL_SILVER_MINERAL) && !silver2Found && recognition.getTop() > 350) {
                    silver2Position = (int) recognition.getLeft();
                    silver2Found = true;
                    i++;
                }
            }
        }
        return i;
    }

    String goldLocation(){
        if(elements == 2) {
            if (silver1Found && silver2Found && !cubeFound)
                return ("right");
            else if (cubeFound && (silver1Found || silver2Found)) {
                if (silver2Found)
                    silver1Position = silver2Position;
                if (goldPosition < silver1Position)
                    return ("left");
                else return ("center");
            }
        }
        else if(elements == 3){
            if(goldPosition < silver1Position && goldPosition < silver2Position)
                return ("left");
            else if(goldPosition > silver1Position && goldPosition > silver2Position)
                return ("right");
            else return ("center");
        }
        else {
            elements = scanGold(tfod.getUpdatedRecognitions());
        }

        return "not_found";
    }

    public class Initialize{

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

        private void motorsInit(){
            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
            liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
            armMotor = hardwareMap.get(DcMotor.class, "armMotor");
            markerServo = hardwareMap.get(Servo.class, "markerServo");

            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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

            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        private void vuforiaInit(){
            VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
            params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            params.vuforiaLicenseKey = "ARDQXkP/////AAABmZbc2Ud2I0b9miNIUDsTMNODX/myN9Y/VqGh0NCsRXeaus19hlL8b//KXthG4HyAyRKxJ2aKBRZ9A000NWP2u4val3zdjTlCeIa3k+xttxFnkPSz3WOe7zhygG6z4FQEvZe3a7H2MNk2f2hUGqibfceIIhoiPzMaMq+bue7z+n0lKs0TDOVofWDlYd8OWqB+PvTegu3imXsgIgAeIEb25fTfN9ke4mD37TUOLunYiDhSNrpk4u8gYikyE5SqkZOSGuI7ifFK09okk85Yhh1C+/FtWkUXD3qZwmLaKNJUeodiNzwekVrXbNtFBFWmhGq3GnF8+RpMH6d2qsX5BdvkFusWFDmfwG/9RaZOmN+CXISP";
            params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.TEAPOT;

            vuforia = ClassFactory.getInstance().createVuforia(params);
            Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 7);
        }

        private void tfodInit() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }
    }

    public class MovingMotor{
        public void stop() {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
        public void forwards(double maxPower, int targetEncoder) {
            stop();

            double initialGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
            currentGyro = initialGyro;

            WheelEncoder wheelEncoder = new WheelEncoder();

            wheelEncoder.start();
            //sleep(1000);
            int initialEncoder = backRightMotor.getCurrentPosition();
            int currentEncoder = backRightMotor.getCurrentPosition() - initialEncoder;
            double errOffCourse;
            double errOffCourse2 = 0;
            double errOffCourseTotal;
            double errDistance;

            double kp = 0.055;
            double kd = 0.0009;
            double distanceCorrection = 0.0002;

            frontLeftMotor.setPower(0.2);
            frontRightMotor.setPower(0.2);
            backLeftMotor.setPower(0.2);
            backRightMotor.setPower(0.2);

            while(currentEncoder <= targetEncoder && opModeIsActive()){

                errOffCourse = currentGyro - initialGyro;

                if(currentEncoder < targetEncoder/2)
                    errDistance = currentEncoder;
                else errDistance = targetEncoder - currentEncoder;

                errOffCourseTotal = errOffCourse*kp + kd*(errOffCourse - errOffCourse2);

                errDistance *= distanceCorrection;

                if((0.2 + errOffCourse + errDistance > maxPower || 0.2 - errOffCourse + errDistance > maxPower) && maxPower != 0){
                    frontLeftMotor.setPower(maxPower + errOffCourseTotal);
                    if(maxPower < 1)
                        frontRightMotor.setPower(maxPower + 0.1 - errOffCourseTotal);
                    else frontRightMotor.setPower(maxPower - errOffCourseTotal);
                    backLeftMotor.setPower(maxPower + errOffCourseTotal);
                    backRightMotor.setPower(maxPower - errOffCourseTotal);
                }
                else {
                    frontLeftMotor.setPower(0.2 + errOffCourseTotal + errDistance);
                    frontRightMotor.setPower(0.3 - errOffCourseTotal + errDistance);
                    backLeftMotor.setPower(0.2 + errOffCourseTotal + errDistance);
                    backRightMotor.setPower(0.2 - errOffCourseTotal + errDistance);
                }

                currentGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
                currentEncoder = backRightMotor.getCurrentPosition() - initialEncoder;
                errOffCourse2 = errOffCourse;

                telemetry.addData("Encoder", currentEncoder);
                telemetry.update();
            }

            stop();

            wheelEncoder.stop();
        }
        public void backwards(double maxPower, int targetEncoder) {
            stop();

            if(maxPower > 0)
                maxPower *= -1;
            if(targetEncoder > 0)
                targetEncoder *= -1;

            WheelEncoder wheelEncoder = new WheelEncoder();
            double initialGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
            currentGyro = initialGyro;

            wheelEncoder.start();

            int initialEncoder = backRightMotor.getCurrentPosition();
            int currentEncoder = backRightMotor.getCurrentPosition() - initialEncoder;
            double errOffCourse;
            double errOffCourse2 = 0;
            double errOffCourseTotal;
            double errDistance;

            double kp = 0.055;
            double kd = 0.0009;
            double distanceCorrection = 0.0002;

            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(-0.2);

            while(currentEncoder >= targetEncoder && opModeIsActive()){

                errOffCourse = currentGyro - initialGyro;

                if(currentEncoder >= targetEncoder/2)
                    errDistance = currentEncoder;
                else errDistance = targetEncoder - currentEncoder;

                errOffCourseTotal =kp*errOffCourse + kd*(errOffCourse - errOffCourse2);

                errDistance *= distanceCorrection;

                if((-0.2 - errOffCourseTotal + errDistance > maxPower || -0.2 + errOffCourseTotal + errDistance > maxPower) && maxPower != 0){
                    if(maxPower > -1)
                        frontRightMotor.setPower(maxPower-0.1 - errOffCourseTotal);
                    else frontRightMotor.setPower(maxPower - errOffCourseTotal);
                    backRightMotor.setPower(maxPower - errOffCourseTotal);
                    frontLeftMotor.setPower(maxPower + errOffCourseTotal);
                    backLeftMotor.setPower(maxPower + errOffCourseTotal);
                }else{
                    frontRightMotor.setPower(-0.3 - errOffCourseTotal + errDistance);
                    backRightMotor.setPower(-0.2 - errOffCourseTotal + errDistance);
                    frontLeftMotor.setPower(-0.2 + errOffCourseTotal + errDistance);
                    backLeftMotor.setPower(-0.2 + errOffCourseTotal + errDistance);
                }

                errOffCourse2 = errOffCourse;
                currentEncoder = backRightMotor.getCurrentPosition() - initialEncoder;
                currentGyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;

                telemetry.addData("Encoder", currentEncoder);
                telemetry.update();
            }
            stop();

            wheelEncoder.stop();
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
