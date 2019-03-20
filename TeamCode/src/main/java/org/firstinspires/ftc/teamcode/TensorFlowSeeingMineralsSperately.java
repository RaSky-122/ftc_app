package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.*;

@TeleOp(name = "TensorFlowJustGold", group = "Learning")

public class TensorFlowSeeingMineralsSperately extends LinearOpMode {

    /*private DcMotor upperRightMotor;
    private DcMotor upperLeftMotor;
    private DcMotor lowerRightMotor;
    private DcMotor lowerLeftMotor;*/

    double power = 0.25;

    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;



    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "ARDQXkP/////AAABmZbc2Ud2I0b9miNIUDsTMNODX/myN9Y/VqGh0NCsRXeaus19hlL8b//KXthG4HyAyRKxJ2aKBRZ9A000NWP2u4val3zdjTlCeIa3k+xttxFnkPSz3WOe7zhygG6z4FQEvZe3a7H2MNk2f2hUGqibfceIIhoiPzMaMq+bue7z+n0lKs0TDOVofWDlYd8OWqB+PvTegu3imXsgIgAeIEb25fTfN9ke4mD37TUOLunYiDhSNrpk4u8gYikyE5SqkZOSGuI7ifFK09okk85Yhh1C+/FtWkUXD3qZwmLaKNJUeodiNzwekVrXbNtFBFWmhGq3GnF8+RpMH6d2qsX5BdvkFusWFDmfwG/9RaZOmN+CXISP";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        /*upperLeftMotor = hardwareMap.get(DcMotor.class, "upperLeftMotor");
        upperRightMotor = hardwareMap.get(DcMotor.class, "upperRightMotor");
        lowerLeftMotor = hardwareMap.get(DcMotor.class, "lowerLeftMotor");
        lowerRightMotor = hardwareMap.get(DcMotor.class, "lowerRightMotor");*/

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }

        while (opModeIsActive()) {

            smoothMovement();

            if (tfod != null) {
                int positionGold;
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if(updatedRecognitions != null){
                    for(Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getTop() > 600 && recognition.getLeft() > 250) {
                            telemetry.addData("Uita-l ba", "yay " + recognition.getLeft() + " " + recognition.getTop());
                            telemetry.update();

                            /*positionGold = (int) recognition.getLeft();
                            while(positionGold > 500){
                                positionGold = (int) recognition.getLeft();

                                upperLeftMotor.setPower(-power);
                                lowerLeftMotor.setPower(-power);
                                upperRightMotor.setPower(power);
                                lowerRightMotor.setPower(power);
                            }
                            while(positionGold < 200){
                                positionGold = (int) recognition.getLeft();

                                upperLeftMotor.setPower(power);
                                lowerLeftMotor.setPower(power);
                                upperRightMotor.setPower(-power);
                                lowerRightMotor.setPower(-power);
                            }*/
                        }
                        else if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                            telemetry.addData("Your princess is in another castle ", "hah " + recognition.getLeft() + " " + recognition.getTop());
                            telemetry.update();
                        }
                        else telemetry.update();
                    }
                }
            }
        }
        if(tfod != null)
            tfod.shutdown();
    }

        private void initVuforia () {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        }

        /**
         * Initialize the Tensor Flow Object Detection engine.
         */
        private void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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
}
