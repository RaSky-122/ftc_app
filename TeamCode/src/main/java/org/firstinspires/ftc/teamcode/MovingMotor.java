package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OnlyWheels", group = "Simple")
@Disabled

public class MovingMotor extends LinearOpMode {

    private DcMotor upperRightMotor;
    private DcMotor upperLeftMotor;
    private DcMotor lowerRightMotor;
    private DcMotor lowerLeftMotor;

    double power = 0.5;

    @Override
    public void runOpMode(){


        upperLeftMotor = hardwareMap.get(DcMotor.class, "upperLeftMotor");
        upperRightMotor = hardwareMap.get(DcMotor.class, "upperRightMotor");
        lowerLeftMotor = hardwareMap.get(DcMotor.class, "lowerLeftMotor");
        lowerRightMotor = hardwareMap.get(DcMotor.class, "lowerRightMotor");
        double power;
        upperLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.right_bumper)
                power = 0.15;
            else
                power = 0.25;
            if(gamepad1.y){
                upperLeftMotor.setPower(power*2);
                lowerRightMotor.setPower(power*2);
                lowerLeftMotor.setPower(0);
                upperRightMotor.setPower(0);
            }
            else if(gamepad1.x){
                upperLeftMotor.setPower(0);
                lowerRightMotor.setPower(0);
                lowerLeftMotor.setPower(power*2);
                upperRightMotor.setPower(power*2);
            }
            else if(gamepad1.b){
                upperLeftMotor.setPower(0);
                lowerRightMotor.setPower(0);
                lowerLeftMotor.setPower(-power*2);
                upperRightMotor.setPower(-power*2);
            }
            else if(gamepad1.a){
                upperLeftMotor.setPower(-power*2);
                lowerRightMotor.setPower(-power*2);
                lowerLeftMotor.setPower(0);
                upperRightMotor.setPower(0);
            }
            else if(gamepad1.dpad_up){
                upperLeftMotor.setPower(power);
                upperRightMotor.setPower(power);
                lowerLeftMotor.setPower(power);
                lowerRightMotor.setPower(power);
            }
            else if(gamepad1.dpad_down){
                upperRightMotor.setPower(-power);
                upperLeftMotor.setPower(-power);
                lowerRightMotor.setPower(-power);
                lowerLeftMotor.setPower(-power);
            }
            else if(gamepad1.dpad_right){
                upperLeftMotor.setPower(power*2);
                lowerLeftMotor.setPower((-power)*2);
                upperRightMotor.setPower((-power)*2);
                lowerRightMotor.setPower(power*2);
            }
            else if(gamepad1.dpad_left){
                upperLeftMotor.setPower((-power)*2);
                lowerLeftMotor.setPower(power*2);
                upperRightMotor.setPower(power*2);
                lowerRightMotor.setPower((-power)*2);
            }
            else if(gamepad1.left_stick_x>0){
                upperLeftMotor.setPower(power);
                lowerLeftMotor.setPower(power);
                upperRightMotor.setPower(-power);
                lowerRightMotor.setPower(-power);
            }
            else if(gamepad1.left_stick_x<0){
                upperLeftMotor.setPower(-power);
                lowerLeftMotor.setPower(-power);
                upperRightMotor.setPower(power);
                lowerRightMotor.setPower(power);
            }
            else{
                upperLeftMotor.setPower(0);
                lowerLeftMotor.setPower(0);
                upperRightMotor.setPower(0);
                lowerRightMotor.setPower(0);
            }
        }
    }

}
