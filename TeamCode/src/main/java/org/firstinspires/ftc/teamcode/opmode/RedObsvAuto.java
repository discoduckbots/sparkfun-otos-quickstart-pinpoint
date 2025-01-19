package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;

@Config
@Autonomous(name = "RedSpecimenAuto", group = "Autonomous")
public class RedObsvAuto extends DuckbotAuto {

    MecanumDrive drive = null;
    Grabber teleGrabber = null;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        AutoArm arm = new AutoArm(hardwareStore);
        AutoIntake intake = new AutoIntake(hardwareStore);
        AutoGrabber grabber = new AutoGrabber(hardwareStore);

        drive = hardwareStore.getDrive();
        teleGrabber = hardwareStore.getGrabber();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        teleGrabber.closeGrabber();

        waitForStart();
        TranslationalVelConstraint slowVel = new TranslationalVelConstraint(15.0);
        ProfileAccelConstraint slowAccel = new ProfileAccelConstraint(-10, 10);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d scoringPose = new Pose2d(-29, -14.5, Math.toRadians(0));
        Vector2d grabPose = new Vector2d(-15, 23);
        Vector2d closeGrabPose = new Vector2d(-15, 23);
        Vector2d awayFromWallPose = new Vector2d(-15, 21.5);
        Vector2d awayFromBarPose = new Vector2d(-12, 15);


        TrajectoryActionBuilder driveToBar = drive.actionBuilder(initialPose)
               // .splineToLinearHeading(scoringPose, Math.toRadians(180), slowVel, slowAccel);
                .strafeTo(new Vector2d(-29, -14.5)); // move to bar

        TrajectoryActionBuilder ramBar = driveToBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(-35, -14.5)); // arbitrary x value needs tuning - ram bar

        TrajectoryActionBuilder backUpFromBar = ramBar.endTrajectory().fresh()
                .strafeTo(awayFromBarPose);

        TrajectoryActionBuilder pushSamplesToObsZone = backUpFromBar.endTrajectory().fresh()
                //.strafeTo(new Vector2d(-15, 15))//backup a little
                .strafeToLinearHeading(new Vector2d(-54, 23), Math.toRadians(-180))
                .strafeTo(new Vector2d(-14, 23)); //push first sample back
                //.strafeTo(new Vector2d(-54, 37)) //position for second sample
                //.strafeTo(new Vector2d(-3, 37)) //push second sample back
                //.strafeTo(new Vector2d(-54, 42.5))//position for third sample
                //.strafeTo(new Vector2d(-3, 42.5)) //push third sample back

        //TrajectoryActionBuilder firstMoveToWall = pushSamplesToObsZone.endTrajectory().fresh()
        //        .strafeTo(grabPose);

        //TrajectoryActionBuilder pickupFirstSample = firstMoveToWall.endTrajectory().fresh()
        //        .strafeTo(closeGrabPose);

        TrajectoryActionBuilder deliverFirstSample = pushSamplesToObsZone.endTrajectory().fresh()
                .strafeTo(awayFromWallPose) //back away from wall
                .strafeToLinearHeading(new Vector2d(-27, -8.5), Math.toRadians(0))
                .strafeTo(new Vector2d(-29, -8.5));

        TrajectoryActionBuilder firstRamBar = deliverFirstSample.endTrajectory().fresh()
                .strafeTo(new Vector2d(-35, -8.5)); // needs to be tuned/tested

        TrajectoryActionBuilder firstBackUpFromBar = firstRamBar.endTrajectory().fresh()
                .strafeTo(awayFromBarPose);

        TrajectoryActionBuilder secondMoveToWall = firstBackUpFromBar.endTrajectory().fresh()
                .strafeToLinearHeading(grabPose, Math.toRadians(-180));

        TrajectoryActionBuilder pickupSecondSample = secondMoveToWall.endTrajectory().fresh()
                .strafeTo(closeGrabPose);

        TrajectoryActionBuilder deliverSecondSample = pickupSecondSample.endTrajectory().fresh()
                .strafeTo(awayFromWallPose)
                .strafeToLinearHeading(new Vector2d(-27, -4), Math.toRadians(0))
                .strafeTo(new Vector2d(-29, -4));

        TrajectoryActionBuilder secondRamBar = deliverSecondSample.endTrajectory().fresh()
                .strafeTo(new Vector2d(-35, -4)); // needs to be tuned/tested

        TrajectoryActionBuilder thirdMoveToWall = secondRamBar.endTrajectory().fresh()
                .strafeToLinearHeading(grabPose, Math.toRadians(-180));

        TrajectoryActionBuilder pickupThirdSample = thirdMoveToWall.endTrajectory().fresh()
                .strafeTo(closeGrabPose);

        TrajectoryActionBuilder deliverThirdSample = pickupThirdSample.endTrajectory().fresh()
                .strafeTo(awayFromWallPose)
                .strafeToLinearHeading(new Vector2d(-27, -0.5), Math.toRadians(0))
                .strafeTo(new Vector2d(-29, -0.5));

        TrajectoryActionBuilder thirdRamWall = deliverThirdSample.endTrajectory().fresh()
                .strafeTo(new Vector2d(-33, -0.5)); // needs to be tuned/tested


        if (opModeIsActive()) {
            if (isStopRequested()) return;
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                            arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            new SleepAction(0.05),
                            grabber.grabberOut(),
                            new SleepAction(0.1),
                        new ParallelAction(
                                intake.retract(),
                                driveToBar.build(),
                                arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN)
                                )
                        )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.2),
                            ramBar.build(),
                            new SleepAction(0.2),
                            grabber.grabberRelease(),
                            new SleepAction(0.5),
                            //grabber.grabberIn(),
                            backUpFromBar.build()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            intake.retract(),
                            new SequentialAction(
                                    grabber.grabberOut(),
                                    grabber.grabberRelease()
                            ),
                            arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            pushSamplesToObsZone.build()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                            new SleepAction(0.2),
                            arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );
/*
            Actions.runBlocking(
                    new ParallelAction(
                            intake.retract(),
                            firstMoveToWall.build(),
                            new SequentialAction(
                                    grabber.grabberOut(),
                                    new SleepAction(0.2),
                                    grabber.grabberRelease()
                            )
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            intake.retract(),
                            new SequentialAction(
                                    pickupFirstSample.build(),
                                    new SleepAction(0.2),
                                    grabber.grabberGrab(),
                                    new SleepAction(0.2),
                                    arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                            )
                    )
            );
            */
            Actions.runBlocking(
                    new ParallelAction(
                            intake.retract(),
                            deliverFirstSample.build(),
                            arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            firstRamBar.build(),
                            new SleepAction(0.2),
                            grabber.grabberRelease(),
                            new SleepAction(0.5),
                            //grabber.grabberIn(),
                            firstBackUpFromBar.build()

                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            intake.retract(),
                            secondMoveToWall.build(),
                             new SequentialAction(
                                     arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                                     new SleepAction(0.2)
                                     //grabber.grabberOut(),
                                     //new SleepAction(0.2)
                             )
                    )
            );
/*
            Actions.runBlocking(
                    new SequentialAction(
                            pickupSecondSample.build(),
                            new SleepAction(0.5),
                            grabber.grabberGrab(),
                            new SleepAction(0.2),
                            arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            intake.retract(),
                            deliverSecondSample.build(),
                            arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN),
                            grabber.grabberOut()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            secondRamBar.build(),
                            new SleepAction(0.2),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );
/*
            Actions.runBlocking(
                    new ParallelAction(
                            arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            pickupThirdSample.build()
                    )
            ); */

        }
    }
}