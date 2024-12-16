package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;

import java.util.Vector;

@Config
@Autonomous(name = "DUSTIN_TEST_AUTO", group = "Autonomous")
public class DustinTestAuto extends DuckbotAuto{

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

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d scoringPose = new Pose2d(-30, 30, Math.toRadians(0));
        Pose2d grabPose = new Pose2d(-5, -5, Math.toRadians(180));

        TrajectoryActionBuilder driveToBar = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoringPose, Math.toRadians(0));

        TrajectoryActionBuilder pushSamplesToObsZone = driveToBar.endTrajectory().fresh()
                .lineToX(-20) //backup a little
                .splineToLinearHeading(new Pose2d(-30, 20, Math.toRadians(180)), Math.toRadians(180)) //position for first sample
                .lineToX(-5) //push first sample back
                .splineToConstantHeading(new Vector2d(-30, 25), Math.toRadians(180)) //position for second sample
                .lineToX(-5) //push second sample back
                .splineToConstantHeading(new Vector2d(-30, 30), Math.toRadians(180)) //position for third sample
                .lineToX(-5) //push third sample back
                .lineToX(10) //move away from obs zone
                .splineToLinearHeading(grabPose, Math.toRadians(180)); //position to grab sample

        TrajectoryActionBuilder deliverFirstSample = pushSamplesToObsZone.endTrajectory().fresh()
                .splineToLinearHeading(scoringPose, Math.toRadians(0))
                .lineToY(25);

        TrajectoryActionBuilder pickupSecondSample = deliverFirstSample.endTrajectory().fresh()
                .splineToLinearHeading(grabPose, Math.toRadians(180));

        TrajectoryActionBuilder deliverSecondSample = pickupSecondSample.endTrajectory().fresh()
                .splineToLinearHeading(scoringPose, Math.toRadians(0))
                .lineToY(20);

        TrajectoryActionBuilder pickupThirdSample = deliverSecondSample.endTrajectory().fresh()
                .splineToLinearHeading(grabPose, Math.toRadians(180));

        TrajectoryActionBuilder deliverThirdSample = pickupThirdSample.endTrajectory().fresh()
                .splineToLinearHeading(scoringPose, Math.toRadians(0))
                .lineToY(20);

        if (opModeIsActive()) {
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new ParallelAction(
                            driveToBar.build(),
                            arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                            grabber.grabberRelease()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            pushSamplesToObsZone.build()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                            arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            deliverFirstSample.build(),
                            arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                            grabber.grabberRelease()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            pickupSecondSample.build()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                            arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            deliverSecondSample.build(),
                            arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                            grabber.grabberRelease()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                            pickupThirdSample.build()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                            arm.liftToTargetPosition(Arm.LIFT_RAISE_ABOVE_WALL)
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            deliverThirdSample.build(),
                            arm.liftToTargetPosition(Arm.LIFT_ABOVE_BAR)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            arm.liftToTargetPosition(Arm.LIFT_BELOW_BAR),
                            grabber.grabberRelease()
                    )
            );
        }
    }}