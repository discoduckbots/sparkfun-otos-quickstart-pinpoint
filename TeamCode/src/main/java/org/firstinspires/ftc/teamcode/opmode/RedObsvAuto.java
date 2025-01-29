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
        teleGrabber.closeGrabber();
        teleIntake.rotateIntakeTo0();

        waitForStart();
        TranslationalVelConstraint slowVel = new TranslationalVelConstraint(40);
        ProfileAccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Vector2d grabPose = new Vector2d(-20, 23);
        Vector2d closeGrabPose = new Vector2d(-14.5, 23);

        TrajectoryActionBuilder deliverPreload = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-20, -12.7)); // move to bar

        TrajectoryActionBuilder ramBarForPreload = deliverPreload.endTrajectory().fresh()
                .strafeTo(new Vector2d(-31.4, -12.7));

        TrajectoryActionBuilder backUpFromBar = ramBarForPreload.endTrajectory().fresh()
                .strafeTo(new Vector2d(-22, -12.7))
                .strafeToLinearHeading(new Vector2d(-27, 13.4), Math.toRadians(-180));

        TrajectoryActionBuilder grabFirst = backUpFromBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(-30, 25.7))
                .strafeTo(new Vector2d(-38, 25.7));

        TrajectoryActionBuilder deliverFirst = grabFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-10, 38), Math.toRadians(0));

        TrajectoryActionBuilder grabSecond = deliverFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-27, 38), Math.toRadians(-180))
                .strafeTo(new Vector2d(-38, 35));

        TrajectoryActionBuilder deliverSecond = grabSecond.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-10, 42), Math.toRadians(0));

        TrajectoryActionBuilder firstMoveToWall = deliverSecond.endTrajectory().fresh()
                //.strafeTo(new Vector2d(-20, 30))
                .strafeToLinearHeading((grabPose), Math.toRadians(-180));

        TrajectoryActionBuilder firstGrabFromWall = firstMoveToWall.endTrajectory().fresh()
                .strafeTo(closeGrabPose);

        //TrajectoryActionBuilder firstMoveToWall = pushSamplesToObsZone.endTrajectory().fresh()
        //        .strafeTo(grabPose);

        //TrajectoryActionBuilder pickupFirstSample = firstMoveToWall.endTrajectory().fresh()
        //        .strafeTo(closeGrabPose);

       /* TrajectoryActionBuilder deliverFirstSample = pushSamplesToObsZone.endTrajectory().fresh()
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

*/
        if (opModeIsActive()) {
            if (isStopRequested()) return;

// Score Preload Specimen
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                        new ParallelAction(
                                deliverPreload.build(),
                                arm.liftToTargetPosition(Arm.LIFT_PLACE_SPECIMEN),
                                grabber.grabberOut()
                        ),
                            new SleepAction(0.05),
                            ramBarForPreload.build(),
                            new SleepAction(0.1),
                            grabber.grabberRelease()
                    )
            );
// Push First Sample
            Actions.runBlocking(
                    new SequentialAction(
                            backUpFromBar.build(),
                            grabber.grabberIn(),
                            new ParallelAction(
                                    intake.retract(),
                                    arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                                    grabFirst.build(),
                                    intake.intakeDown(),
                                    intake.intakeOpen()
                            ),
                            new SleepAction(0.2),
                            intake.intakeClose(),
                            new SleepAction(0.2),
                            deliverFirst.build(),
                            intake.intakeOpen(),
                            new SleepAction(0.1)
                    )
            );
// Push Second Sample
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    intake.retract(),
                                    arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                                    grabSecond.build(),
                                    intake.intakeDown(),
                                    intake.intakeOpen()
                            ),
                            new SleepAction(0.2),
                            intake.intakeClose(),
                            new SleepAction(0.2),
                            deliverSecond.build(),
                            intake.intakeOpen()
                    )
            );
//Grab first specimen from wall
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    firstMoveToWall.build(),
                                    intake.intakeUp(),
                                    new SequentialAction(
                                            grabber.grabberOut(),
                                            new SleepAction(0.1),
                                            grabber.grabberRelease()
                                            )
                            ),
                            firstGrabFromWall.build(),
                            new SleepAction(0.2),
                            grabber.grabberGrab(),
                            new SleepAction(0.2)
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