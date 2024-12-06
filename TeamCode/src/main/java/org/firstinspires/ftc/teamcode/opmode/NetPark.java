package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;


@Autonomous(name = "netPark", group = "Robot")
public class NetPark extends LinearOpMode {
    Arm arm = null;
    Grabber grabber = null;
    MecanumDrive drive = null;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        arm = hardwareStore.getArm();
        drive = hardwareStore.getDrive();
//        grabber = hardwareStore.getGrabber();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Pose2d startPose = new Pose2d(0, 0, 0);
        Vector2d firstMove = new Vector2d(90, 0);


        waitForStart();

        if (opModeIsActive()) {

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToConstantHeading(new Vector2d(30, 30), 0)
                            .strafeTo(new Vector2d(60, 30))
                            .strafeTo(new Vector2d(70, -30))
                            .splineTo(new Vector2d(0, 0), 0,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
            );


            //splineToConstantHeading with Slowdown
//            Actions.runBlocking(
//                    drive.actionBuilder(startPose)
//                            .splineToConstantHeading(new Vector2d(30, 30), 0,
//                                    new TranslationalVelConstraint(20.0),
//                                    new ProfileAccelConstraint(-10, 10))
//                            .build()
//            );

            //splineToConstantHeading
//            Actions.runBlocking(
//                    drive.actionBuilder(startPose)
//                            .splineToConstantHeading(new Vector2d(30, 30), 0)
//                            .build()
//            );

            //splineTo
//            Actions.runBlocking(
//                    drive.actionBuilder(startPose)
//                            .splineTo(new Vector2d(30,30), Math.PI / 2)
//                            .build()
//            );


            //strafeTo
//            Actions.runBlocking(
//                    drive.actionBuilder(startPose)
//                            .strafeTo(firstMove, null, null)
//                            .build());
        }
    }
}