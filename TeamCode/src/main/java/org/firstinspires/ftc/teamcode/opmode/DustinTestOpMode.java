package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Dustin Test Op Mode", group= "Linear Opmode")
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

            

//            double pos = 0.5 + (-gamepad1.left_stick_y * 0.5);
//
//            testServo.setPosition(pos);

            telemetry.addData("servo-pos", testServo.getPosition());
            telemetry.addData("LS-x - LS-y", gamepad1.left_stick_x + ", " + (gamepad1.left_stick_y*-1));
            telemetry.update();




            if (gamepad1.square){
               testServo.setPosition(0.15);
           }
            if (gamepad1.cross){
               testServo.setPosition(0.2);
           }

            if (gamepad1.circle){
               testServo.setPosition(0.25);
           }

            if (gamepad1.triangle){
                testServo.setPosition(0.3);
            }

            if (gamepad1.left_bumper){
                testServo.setPosition(0.35);
            }

            if (gamepad1.left_trigger > 0.1){ //closed
                testServo.setPosition(0.4);
            }

            if (gamepad1.right_bumper){
                testServo.setPosition(0.45);
            }

            if (gamepad1.right_trigger > 0.5){ //open
                testServo.setPosition(1);
            }
        }

    }

}
