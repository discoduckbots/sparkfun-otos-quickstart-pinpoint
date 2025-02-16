package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.SampleDetector;
import org.openftc.easyopencv.OpenCvCamera;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="OpenCv Test Op Mode", group= "Linear Opmode")
public class OpenCvTestOpMode extends LinearOpMode {

    OpenCvCamera webcam = null;
    private SampleDetector sampleDetector = null;

    @Override
    public void runOpMode() throws InterruptedException {
       // webcam = hardwareMap.get(OpenCvCamera.class, "webcam");
        sampleDetector = new SampleDetector(hardwareMap.get(WebcamName.class, "webcam"), hardwareMap);

        waitForStart();

        while (opModeIsActive()) {


        }

    }

}
