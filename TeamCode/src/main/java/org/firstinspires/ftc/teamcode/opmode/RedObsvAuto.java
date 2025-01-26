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
        TranslationalVelConstraint slowVel = new TranslationalVelConstraint(40);
        ProfileAccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Vector2d grabPose = new Vector2d(-15, 23);
        Vector2d closeGrabPose = new Vector2d(-15, 23);
        Vector2d awayFromWallPose = new Vector2d(-15, 21.5);
        Vector2d awayFromBarPose = new Vector2d(-12, 15);


        TrajectoryActionBuilder deliverPreload = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-20, -12.7)); // move to bar

        TrajectoryActionBuilder ramBarForPreload = deliverPreload.endTrajectory().fresh()
                .strafeTo(new Vector2d(-31.4, -12.7));

        TrajectoryActionBuilder backUpFromBar = ramBarForPreload.endTrajectory().fresh()
                .strafeTo(new Vector2d(-20, -12.7));


        TrajectoryActionBuilder pushSamplesToObsZone = backUpFromBar.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-23, 10), Math.toRadians(180)) // move over and turn 180
                .strafeTo(new Vector2d(-56.5, 28.8)); //to first push position
                //.strafeTo(new Vector2d(-12.5, 28.8)); //push first sample back
                //.strafeTo(new Vector2d(-56.5, 36)) //position for second sample
                //.strafeTo(new Vector2d(-12.5, 36)) //push second sample back
                //.strafeTo(new Vector2d(-56, 42.5))//position for third sample
                //.strafeTo(new Vector2d(-12.5, 42.5)) //push third sample back

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
// Score Preload Speciment
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
// Push Samples
            Actions.runBlocking(
                    new SequentialAction(
                            backUpFromBar.build(),
                            grabber.grabberIn(),
                            new ParallelAction(
                                    arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                                    pushSamplesToObsZone.build()
                            )
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