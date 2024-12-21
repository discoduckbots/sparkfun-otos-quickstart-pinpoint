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
@Autonomous(name = "BlueObsvAuto", group = "Autonomous")
public class RedNetAuto extends DuckbotAuto {

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
        Vector2d scoringPose = new Vector2d(-29, -14.5); //arbitrary value
        Vector2d closeScoringPose = new Vector2d(14, 26); //arbitrary value
        double scoringHeading = Math.toRadians(45);

        TrajectoryActionBuilder driveToFirstGrabPos = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-29, -14.5)); // strafe to first grab position

        TrajectoryActionBuilder scoreFirstSample = driveToFirstGrabPos.endTrajectory().fresh()
                .strafeToLinearHeading(scoringPose, scoringHeading) //strafe to scoring position
                .strafeTo(closeScoringPose); //move a little closer

        TrajectoryActionBuilder driveToSecondGrabPosition = scoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-26, -8.5), Math.toRadians(0)); //strafe to second grab position

        TrajectoryActionBuilder scoreSecondSample = driveToSecondGrabPosition.endTrajectory().fresh()
                .strafeToLinearHeading(scoringPose, scoringHeading) //strafe to scoring position
                .strafeTo(closeScoringPose); //move a little closer

        TrajectoryActionBuilder driveToThirdGrabPosition = scoreSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-26, -4), Math.toRadians(0));

        TrajectoryActionBuilder scoreThirdSample = driveToThirdGrabPosition.endTrajectory().fresh()
                .strafeToLinearHeading(scoringPose, scoringHeading) //strafe to scoring position
                .strafeTo(closeScoringPose); //move a little closer

        TrajectoryActionBuilder driveToPark = scoreThirdSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-26, -0.5), Math.toRadians(0)) // back up from basket and turn
                .strafeTo(new Vector2d(-29, -0.5)); // strafe to chamber


        if (opModeIsActive()) {
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new ParallelAction(
                            driveToFirstGrabPos.build(),
                            intake.intakeIn()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.flip
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