package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.InstantAction;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;

@Config
@Autonomous(name = "RedSpecimenAuto", group = "Autonomous")
public class RedObsvAuto extends DuckbotAuto {

    MecanumDrive drive = null;
    Grabber teleGrabber = null;

    Intake teleIntake = null;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        AutoArm arm = new AutoArm(hardwareStore);
        AutoIntake intake = new AutoIntake(hardwareStore);
        AutoGrabber grabber = new AutoGrabber(hardwareStore);

        drive = hardwareStore.getDrive();
        teleGrabber = hardwareStore.getGrabber();
        teleIntake = hardwareStore.getIntake();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        teleGrabber.autoCloseGrabber();
        teleIntake.rotateIntakeTo0();
        teleGrabber.autoCloseGrabber();

        TrajectoryActionBuilder deliverPreload = drive.actionBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(-20, -12.7)); // move to bar

        TrajectoryActionBuilder ramBarForPreload = deliverPreload.endTrajectory().fresh()
                .strafeTo(new Vector2d(-33, -12.7));

        TrajectoryActionBuilder backUpFromBar = ramBarForPreload.endTrajectory().fresh()
                .strafeTo(new Vector2d(-22, -12.7))
                .strafeToLinearHeading(new Vector2d(-27, 13.4), Math.toRadians(-180));

        TrajectoryActionBuilder grabFirst = backUpFromBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(-30, 26))
                .strafeTo(new Vector2d(-37, 26.5));

        TrajectoryActionBuilder deliverFirst = grabFirst.endTrajectory().fresh()
                .strafeTo(new Vector2d(-18, 26.5));

        TrajectoryActionBuilder grabSecond = deliverFirst.endTrajectory().fresh()
                .strafeTo(new Vector2d(-27, 37.5))
                .strafeTo(new Vector2d(-37, 37.5));

        TrajectoryActionBuilder deliverSecond = grabSecond.endTrajectory().fresh()
                .strafeTo(new Vector2d(-18, 35));

        TrajectoryActionBuilder firstGrabFromWall = deliverSecond.endTrajectory().fresh()
                .strafeTo(new Vector2d(-14, 35));

        TrajectoryActionBuilder firstDriveToBar = firstGrabFromWall.endTrajectory().fresh()
                .strafeTo(new Vector2d(-20, 25)) //back away from wall
                .strafeToLinearHeading(new Vector2d(-20, -8.5), Math.toRadians(0))
                .strafeTo(new Vector2d(-29, -8.5));

        TrajectoryActionBuilder firstRamBar = firstDriveToBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(-33, -8.5));

        TrajectoryActionBuilder firstBackUpFromBar = firstRamBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(-32, -8.5));

        TrajectoryActionBuilder secondGrabFromWall = firstBackUpFromBar.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-20,23), Math.toRadians(-180))
                .strafeTo(new Vector2d(-14, 23));

        TrajectoryActionBuilder secondDriveToBar = secondGrabFromWall.endTrajectory().fresh()
                .strafeTo(new Vector2d(-20, 16)) //back away from wall
                .strafeToLinearHeading(new Vector2d(-20, -4), Math.toRadians(0))
                .strafeTo(new Vector2d(-29, -4));

        TrajectoryActionBuilder secondRamBar = secondDriveToBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(-33, -4));

        TrajectoryActionBuilder secondBackUpFromBar = firstRamBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(-20, -4));

        TrajectoryActionBuilder driveToPark = secondRamBar.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(-20, 23), Math.toRadians(-180));

        telemetry.addData("ready to start", "");
        telemetry.update();

        Actions.runBlocking(new SequentialAction(
                 grabber.grabberMiddle(),
                grabber.grabberGrab()
                )
        );

        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) return;

// Drive to Bar Preload
            Actions.runBlocking(
                        new ParallelAction(
                                grabber.grabberGrab(),
                                deliverPreload.build(),
                                arm.liftToTargetPosition(Arm.LIFT_PLACE_PRELOAD_SPECIMEN),
                                grabber.grabberOut()
                        )
            );

// Ram bar preload
            Actions.runBlocking(
                    new SequentialAction(
                            ramBarForPreload.build(),
                            new SleepAction(0.1),
                            grabber.grabberRelease()
                    )
            );

//Back up from bar
            Actions.runBlocking(
                    new SequentialAction(
                            backUpFromBar.build(),
                            grabber.grabberIn()
                    )
            );

// Go to Grab First Sample
            Actions.runBlocking(
                    new ParallelAction(
                            arm.liftToTargetPosition(0),
                            grabFirst.build(),
                            intake.intakeDown()
                    )
            );

 // Grab First sample
            Actions.runBlocking(
                    new SequentialAction(
                            intake.intakeClose(),
                            new SleepAction(0.4)
                    )
            );

    // deliver first sample
            Actions.runBlocking(
                    new ParallelAction(
                            deliverFirst.build(),
                            grabber.grabberRelease(),
                            new SequentialAction(
                                    intake.intakeUp(),
                                    new SleepAction(0.5),
                                    grabber.grabberGrab(),
                                    new SleepAction(0.2),
                                    intake.intakeOpen()
                            )
                    )
            );
//drop first sample
            Actions.runBlocking(
                    new SequentialAction(
                            arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            grabber.grabberOut(),
                            new SleepAction(0.4),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );

            // Go to Grab Second Sample
            Actions.runBlocking(
                    new ParallelAction(
                            arm.liftToTargetPosition(0),
                            grabSecond.build(),
                            intake.intakeDown(),
                            grabber.grabberIn()
                    )
            );

            // Grab Second sample
            Actions.runBlocking(
                    new SequentialAction(
                            intake.intakeClose(),
                            new SleepAction(0.4)
                    )
            );

            // deliver second sample
            Actions.runBlocking(
                    new ParallelAction(
                            deliverSecond.build(),
                            grabber.grabberRelease(),
                            new SequentialAction(
                                    intake.intakeUp(),
                                    new SleepAction(0.5),
                                    grabber.grabberGrab(),
                                    new SleepAction(0.3),
                                    intake.intakeOpen()
                            )
                    )
            );

            //drop second sample
            Actions.runBlocking(
                    new SequentialAction(
                            arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            grabber.grabberOut(),
                            new SleepAction(0.3),
                            grabber.grabberRelease(),
                            new SleepAction(0.2)
                    )
            );


//Grab first specimen from wall
            Actions.runBlocking(
                    new SequentialAction(
                            firstGrabFromWall.build(),
                            new SleepAction(0.2),
                            grabber.grabberGrab(),
                            new SleepAction(0.2)
                    )
            );

//Score first specimen on bar
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    firstDriveToBar.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN),
                                    grabber.grabberOut()
                            ),
                            firstRamBar.build(),
                            new SleepAction(0.1),
                            grabber.grabberRelease()
                    )
            );

            //first back up from bar
            Actions.runBlocking(
                    new ParallelAction(
                            firstBackUpFromBar.build()
                    )
            );

            //Grab second specimen from wall
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    secondGrabFromWall.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL)
                                    ),
                            new SleepAction(0.2),
                            grabber.grabberGrab(),
                            new SleepAction(0.2)
                    )
            );

//Score second specimen on bar
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    secondDriveToBar.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN),
                                    grabber.grabberOut()
                            ),
                            secondRamBar.build(),
                            new SleepAction(0.1),
                            grabber.grabberRelease()
                    )
            );

            //second back up from bar
            Actions.runBlocking(
                    new SequentialAction(
                            secondBackUpFromBar.build(),
                            grabber.grabberMiddle()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    driveToPark.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL)
                            )
                            //new SleepAction(0.2),
                            //grabber.grabberGrab(),
                            //new SleepAction(0.2)
                    )
            );





  /*
 //grab first specimen from wall
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberOut(),
                            pickUpFirstSpecimen.build(), // made up trajectory
                            new SleepAction(0.2),
                            grabber.grabberGrab(),
                            new SleepAction(0.2),
                            arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );

// move to place specimen 1
            Actions.runBlocking(
                    new ParallelAction(
                            deliverFirstSample.build(),
                            arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN)
                    )
            );

 //score specimen 1
            Actions.runBlocking(
                    new SequentialAction(
                            firstRamBar.build(),
                            new SleepAction(0.2),
                            grabber.grabberRelease(),
                            new SleepAction(0.5),
                            firstBackUpFromBar.build()
                    )
            ); */

         }
    }
}