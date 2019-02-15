package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "Learning")
@Disabled

public class TestingHubIMU extends LinearOpMode {

    private BNO055IMU imu;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    Orientation imuGyro = new Orientation();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    VuforiaLocalizer vuforia;

    TFObjectDetector tfod;

    double power = 0.3;

    boolean redImg = false, blueImg = false, frontImg = false, backImg = false;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ARDQXkP/////AAABmZbc2Ud2I0b9miNIUDsTMNODX/myN9Y/VqGh0NCsRXeaus19hlL8b//KXthG4HyAyRKxJ2aKBRZ9A000NWP2u4val3zdjTlCeIa3k+xttxFnkPSz3WOe7zhygG6z4FQEvZe3a7H2MNk2f2hUGqibfceIIhoiPzMaMq+bue7z+n0lKs0TDOVofWDlYd8OWqB+PvTegu3imXsgIgAeIEb25fTfN9ke4mD37TUOLunYiDhSNrpk4u8gYikyE5SqkZOSGuI7ifFK09okk85Yhh1C+/FtWkUXD3qZwmLaKNJUeodiNzwekVrXbNtFBFWmhGq3GnF8+RpMH6d2qsX5BdvkFusWFDmfwG/9RaZOmN+CXISP";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 7);

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
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
        }

        imuInit();

        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }
        }

        beacons.activate();
        /**********************************************************************************************************************/
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        while(opModeIsActive()){

            /*
            telemetry.addData("Linear Acceleration ", imu.getLinearAcceleration());
            telemetry.addData("Aceeleration ", imu.getAcceleration());
            telemetry.addData("Overall Acceleration ", imu.getOverallAcceleration());
            telemetry.addData("Angular Velocity ", imu.getAngularVelocity().toString());
            telemetry.addData("Velocity ", imu.getVelocity());
            telemetry.addData("Position ", imu.getPosition());
            telemetry.addData("Angular Orientation ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES));
            telemetry.update();
            */

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            while(!red.isVisible() && !blue.isVisible() && !back.isVisible() && !front.isVisible()){
                frontRightMotor.setPower(-power);
                frontLeftMotor.setPower(-power);
                backRightMotor.setPower(power);
                backLeftMotor.setPower(power);
            }
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            if(red.isVisible())
                redImg = true;
            else if(blue.isVisible())
                blueImg = true;
            else if(front.isVisible())
                frontImg = true;
            else backImg = true;

            if(updatedRecognitions != null){

            }
        }
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void imuInit(){
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
    }
}
