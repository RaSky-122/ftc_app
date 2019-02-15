package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled

public class ContinuousArmBaseServo extends LinearOpMode {

    private CRServo armBaseServo;

    public ElapsedTime servoLimiter;

    public void setServoLimiter() {this.servoLimiter = new ElapsedTime();}


    public void runOpMode() {

        setServoLimiter();
        boolean servoOn = false;

        double limit = 0;

        armBaseServo = hardwareMap.get(CRServo.class, "armBaseServo");

        while(opModeIsActive()){

            if(gamepad2.dpad_right && servoLimiter.time() + limit <= 5.08){
                armBaseServo.setPower(1);
                servoOn = true;
                servoLimiter.reset();
            }
            else if(gamepad2.dpad_left && servoLimiter.time() + limit <= 5.08) {
                armBaseServo.setPower(-1);
                servoOn = true;
            }
            else if(servoLimiter.time() + limit >= 5.08 && !gamepad2.dpad_left && !gamepad2.dpad_right) {
                armBaseServo.setPower(0);
                servoOn = false;
                limit = 5.08 - servoLimiter.time();
            }
        }
    }
}
