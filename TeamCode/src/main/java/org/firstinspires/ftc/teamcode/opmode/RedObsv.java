package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;

@Disabled
@Autonomous(name = "redObsv", group = "Robot")
public class RedObsv extends LinearOpMode {
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
        Pose2d placePreloadPos = new Pose2d(-28, -10.7, 0);
        Pose2d backFromChamberPos = new Pose2d(-20.1, 29.3, 0);
        Pose2d pushSample1Pos = new Pose2d(-28, -10.7, 0); //has a 180 turn
        Pose2d obsvZone1Pos = new Pose2d(-28, -10.7, 0);
        Pose2d nextToSample2Pos = new Pose2d(-28, -10.7, 0);
        Pose2d pushSample2Pos = new Pose2d(-28, -10.7, 0);
        Pose2d obsvZone2Pos = new Pose2d(-28, -10.7, 0);
        Pose2d nextToSample3Pos = new Pose2d(-28, -10.7, 0);
        Pose2d pushSample3Pos = new Pose2d(-28, -10.7, 0);
        Pose2d obsvZone3Pos = new Pose2d(-28, -10.7, 0);
        Pose2d wallPos = new Pose2d(-28, -10.7, 0);
        Pose2d frontOfChamberPos = new Pose2d(-28, -10.7, 0);
        Pose2d atChamberPos = new Pose2d(-28, -10.7, 0);

        //TrajectoryActionBuilder placePreload = drive.actionBuilder(startPose)
                       // .splineToLinearHeading(placePreloadPos);


        waitForStart();

        if (opModeIsActive()) {
            //lower intake
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToConstantHeading(new Vector2d(22.48, -14.44), Math.toRadians(0.35),
                                    new TranslationalVelConstraint(15.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
            );
            sleep(250);
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(22.48, -14.44, Math.toRadians(0.35)))
                            .splineToConstantHeading(new Vector2d(21.92, -30.61), Math.toRadians(1.69),
                                    new TranslationalVelConstraint(15.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
            );
            sleep(250);
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(21.92, -30.61, Math.toRadians(1.69)))
                            .splineTo(new Vector2d(47.35, -25.84), Math.toRadians(91.38),
                                    new TranslationalVelConstraint(15.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
            );
            sleep(250);
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(47.35, -25.84, Math.toRadians(91.38)))
                            .splineTo(new Vector2d(95.8, -14.67), Math.toRadians(0.1),
                                    new TranslationalVelConstraint(15.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
            );
            /*
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToConstantHeading(new Vector2d(-28, -10.7), 0,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
                    );
            Actions.runBlocking(
                    drive.actionBuilder(pos2)
                            .splineToConstantHeading(new Vector2d(-20.0, -9.14), 0,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(-20.1, 29.3), 0,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
                    );
            Actions.runBlocking(
                    drive.actionBuilder(pos3)
                            .splineTo(new Vector2d(-44.6, 27.9), -Math.toRadians(-180),
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
            ); /*


            /*

                            .splineToConstantHeading(new Vector2d(-7.2, 27.6), -Math.PI,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(-48.6, 28.2), -Math.PI,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(-48.2, 37.3), Math.PI,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(-7.2, 36.5), Math.PI,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(-7.4, 28.6), Math.PI,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(1.26, 28.2), Math.PI,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(-30.4, -2.5), 0,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(1.6, 20.0), Math.PI,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(-30.1, 1.2), 0,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(1.5, 15.85), Math.PI,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .splineToConstantHeading(new Vector2d(-30.0, 4.46), 0,
                                    new TranslationalVelConstraint(20.0),
                                    new ProfileAccelConstraint(-10, 10))
                            .build()
            ); */


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