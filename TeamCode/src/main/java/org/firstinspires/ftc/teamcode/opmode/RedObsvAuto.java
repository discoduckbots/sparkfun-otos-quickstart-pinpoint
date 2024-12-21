package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;

@Config
@Autonomous(name = "RedObsvAuto", group = "Autonomous")
public class RedObsvAuto extends DuckbotAuto {

    MecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        AutoArm arm = new AutoArm(hardwareStore);
        AutoIntake intake = new AutoIntake(hardwareStore);
        AutoGrabber grabber = new AutoGrabber(hardwareStore);

        drive = hardwareStore.getDrive();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        TranslationalVelConstraint slowVel = new TranslationalVelConstraint(15.0);
        ProfileAccelConstraint slowAccel = new ProfileAccelConstraint(-10, 10);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d scoringPose = new Pose2d(-29, -14.5, Math.toRadians(0));
        Vector2d grabPose = new Vector2d(-3, 21.5);
        Vector2d closeGrabPose = new Vector2d(0.5, 21.5);
        Vector2d awayFromWallPose = new Vector2d(-5, 21.5);


        TrajectoryActionBuilder driveToBar = drive.actionBuilder(initialPose)
               // .splineToLinearHeading(scoringPose, Math.toRadians(180), slowVel, slowAccel);
                .strafeTo(new Vector2d(-29, -14.5));

        TrajectoryActionBuilder pushSamplesToObsZone = driveToBar.endTrajectory().fresh()
                .strafeTo(new Vector2d(-10, 15)) //backup a little
                .strafeToLinearHeading(new Vector2d(-54, 30), Math.toRadians(-180))
                .strafeTo(new Vector2d(-54, 26.5)) //position for first sample
                .strafeTo(new Vector2d(-3, 26.5)) //push first sample back
                .strafeTo(new Vector2d(-32, 26.5)) //little forward
                .strafeTo(new Vector2d(-54, 37)) //position for second sample
                .strafeTo(new Vector2d(-3, 37)) //push second sample back
                //.strafeTo(new Vector2d(-54, 42.5))//position for third sample
                //.strafeTo(new Vector2d(-3, 42.5)) //push third sample back
                .strafeTo(grabPose) // position to grab sample
                .strafeTo(closeGrabPose); //get close to wall

        TrajectoryActionBuilder deliverFirstSample = pushSamplesToObsZone.endTrajectory().fresh()
                .strafeTo(awayFromWallPose) //back away from wall
                .strafeToLinearHeading(new Vector2d(-26, -8.5), Math.toRadians(0))
                .strafeTo(new Vector2d(-29, -8.5));

        TrajectoryActionBuilder pickupSecondSample = deliverFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(grabPose, Math.toRadians(-180))
                .strafeTo(closeGrabPose);

        TrajectoryActionBuilder deliverSecondSample = pickupSecondSample.endTrajectory().fresh()
                .strafeTo(awayFromWallPose)
                .strafeToLinearHeading(new Vector2d(-26, -4), Math.toRadians(0))
                .strafeTo(new Vector2d(-29, -4));

        TrajectoryActionBuilder pickupThirdSample = deliverSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(grabPose, Math.toRadians(-180))
                .strafeTo(closeGrabPose);

        TrajectoryActionBuilder deliverThirdSample = pickupThirdSample.endTrajectory().fresh()
                .strafeTo(awayFromWallPose)
                .strafeToLinearHeading(new Vector2d(-26, -0.5), Math.toRadians(0))
                .strafeTo(new Vector2d(-29, -0.5));


        if (opModeIsActive()) {
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new ParallelAction(
                            driveToBar.build()
                            //arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            //arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                            grabber.grabberRelease()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            //arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            pushSamplesToObsZone.build()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab()
                            //arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            deliverFirstSample.build()
                            //arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            //arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                            grabber.grabberRelease()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            //arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            pickupSecondSample.build()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            //grabber.grabberGrab(),
                            //arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            deliverSecondSample.build()
                            //arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            //arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                            grabber.grabberRelease()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            //arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            pickupThirdSample.build()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            //.grabberGrab(),
                            //arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            deliverThirdSample.build()
                            //arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            //arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                            grabber.grabberRelease()
                    )
            );
        }
    }}