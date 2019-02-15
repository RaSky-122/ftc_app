package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.*;

@Autonomous(name = "TensorFlowJustGold", group = "Learning")

public class TensorFlowSeeingMineralsSperately extends LinearOpMode {

    /*private DcMotor upperRightMotor;
    private DcMotor upperLeftMotor;
    private DcMotor lowerRightMotor;
    private DcMotor lowerLeftMotor;*/

    double power = 0.25;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "ARDQXkP/////AAABmZbc2Ud2I0b9miNIUDsTMNODX/myN9Y/VqGh0NCsRXeaus19hlL8b//KXthG4HyAyRKxJ2aKBRZ9A000NWP2u4val3zdjTlCeIa3k+xttxFnkPSz3WOe7zhygG6z4FQEvZe3a7H2MNk2f2hUGqibfceIIhoiPzMaMq+bue7z+n0lKs0TDOVofWDlYd8OWqB+PvTegu3imXsgIgAeIEb25fTfN9ke4mD37TUOLunYiDhSNrpk4u8gYikyE5SqkZOSGuI7ifFK09okk85Yhh1C+/FtWkUXD3qZwmLaKNJUeodiNzwekVrXbNtFBFWmhGq3GnF8+RpMH6d2qsX5BdvkFusWFDmfwG/9RaZOmN+CXISP";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

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

            if (tfod != null) {
                int positionGold;
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if(updatedRecognitions != null){
                    for(Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getTop() > 600 && recognition.getLeft() > 250) {
                            telemetry.addData("Uita-l ba", "yay " + recognition.getLeft());
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
                        else {
                            telemetry.addData("Your princess is in another castle ", "hah " + recognition.getLeft());
                            telemetry.update();
                        }

                    }
                }
            }
        }
        if(tfod != null)
            tfod.shutdown();
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(false);
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
}
