package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Dustin Test Op Mode", group= "Linear Opmode")
@Disabled
public class DustinTestOpMode extends LinearOpMode {

    Servo testServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        testServo = hardwareMap.get(Servo.class, "testServo");

        waitForStart();

        while (opModeIsActive()) {


            //value between zero and one
            /*
            -1 = 0
            0 = 0.5
            1 = 1


            ^ = 0.5
            <- = 0.25
            -> = 0.8
            V = 0

             */

            

            double pos = 0.5 + (-gamepad1.left_stick_y * 0.5);

            testServo.setPosition(pos);

            telemetry.addData("servo-pos", pos);
            telemetry.addData("LS-x - LS-y", gamepad1.left_stick_x + ", " + (gamepad1.left_stick_y*-1));
            telemetry.update();




            if (gamepad1.x){
               testServo.setPosition(0);
           }
            if (gamepad1.a){
               testServo.setPosition(0.5);
           }

            if (gamepad1.b){
               testServo.setPosition(1);
           }
        }

    }

}
