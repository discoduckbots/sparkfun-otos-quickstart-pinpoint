package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
@Autonomous(name = "RedNetAuto", group = "Autonomous")
public class RedNetAuto extends DuckbotAuto {

    MecanumDrive drive = null;
    Grabber teleGrabber = null;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        AutoArm arm = new AutoArm(hardwareStore);
        AutoIntake intake = new AutoIntake(hardwareStore);
        AutoGrabber grabber = new AutoGrabber(hardwareStore);

        teleGrabber = hardwareStore.getGrabber();
        drive = hardwareStore.getDrive();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        teleGrabber.grab();

        waitForStart();
        TranslationalVelConstraint slowVel = new TranslationalVelConstraint(15.0);
        ProfileAccelConstraint slowAccel = new ProfileAccelConstraint(-10, 10);
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Vector2d scoringPose = new Vector2d(8, 17);
        Vector2d closeScoringPose = new Vector2d(14, 26); //arbitrary value
        double scoringHeading = Math.toRadians(-45);

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(scoringPose, scoringHeading); //strafe to score preload

        TrajectoryActionBuilder driveToFirstGrabPos = scorePreload.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(31, 8.8), Math.toRadians(0)); // strafe to first grab position

        TrajectoryActionBuilder scoreFirstSample = driveToFirstGrabPos.endTrajectory().fresh()
                .strafeToLinearHeading(scoringPose, scoringHeading); //strafe to scoring position
                //.strafeTo(closeScoringPose); //move a little closer

        TrajectoryActionBuilder driveToSecondGrabPosition = scoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(31, 17.5), Math.toRadians(0)); //strafe to second grab position

        TrajectoryActionBuilder scoreSecondSample = driveToSecondGrabPosition.endTrajectory().fresh()
                .strafeToLinearHeading(scoringPose, scoringHeading); //strafe to scoring position
                //.strafeTo(closeScoringPose); //move a little closer

        TrajectoryActionBuilder driveToThirdGrabPosition = scoreSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-26, -4), Math.toRadians(0));

        TrajectoryActionBuilder scoreThirdSample = driveToThirdGrabPosition.endTrajectory().fresh()
                .strafeToLinearHeading(scoringPose, scoringHeading); //strafe to scoring position
                //.strafeTo(closeScoringPose); //move a little closer

        TrajectoryActionBuilder driveToPark = scoreThirdSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(44, -7), Math.toRadians(90)) // back up from basket and turn
                .strafeTo(new Vector2d(44, -23.5)); // strafe to chamber //linear slide touch bar pos: 1250


        if (opModeIsActive()) {
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                            new ParallelAction(
                                    scorePreload.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_BASKET)
                                    ),
                            new SleepAction(0.5),
                            grabber.grabberOut(),
                            new SleepAction(0.5),
                            grabber.grabberRelease(),
                            new SleepAction(0.5)
                    )

            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberIn(),
                            new ParallelAction(
                                    arm.liftToTargetPosition(0),
                                    grabber.intakeDown(),
                                    intake.intakeIn()
                                    ),
                            new ParallelAction(
                                    driveToFirstGrabPos.build()
                                ),
                            intake.stopIntake(),
                            intake.extend(),
                            new SleepAction(0.5),
                            grabber.intakeUp(), // I don't know if this is too much stuff to put into one action/sequence, final 2 (now 3) actions could probably be separated
                            new SleepAction(2.0),
                            intake.retract(),
                            new SleepAction(2.0),
                            grabber.grabberGrab(),
                            new SleepAction(2.0)
                    )
            );

          Actions.runBlocking(
                    new ParallelAction(
                            scoreFirstSample.build(),
                            new SequentialAction(
                                    grabber.intakeDown(),
                                    arm.liftToTargetPosition(Arm.LIFT_BASKET),
                                    grabber.grabberOut(),
                                    new SleepAction(1.0),
                                    grabber.grabberRelease()
                            )
                    )
            );
/*
            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberRelease(),
                            new ParallelAction(
                                    driveToSecondGrabPosition.build(),
                                    new SequentialAction(
                                            grabber.grabberGrab(),
                                            new ParallelAction(
                                                    grabber.grabberIn(),
                                                    arm.liftToTargetPosition(0)
                                            ),
                                            new ParallelAction(
                                                    intake.intakeIn(),
                                                    grabber.grabberRelease()
                                            )
                                    )
                            ),
                            grabber.intakeUp(),
                            grabber.grabberGrab()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            scoreSecondSample.build(),
                            new SequentialAction(
                                    grabber.intakeDown(),
                                    arm.liftToTargetPosition(Arm.LIFT_BASKET),
                                    grabber.grabberOut()
                            )
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberRelease(),
                            new ParallelAction(
                                    driveToThirdGrabPosition.build(),
                                    new SequentialAction(
                                            grabber.grabberGrab(),
                                            new ParallelAction(
                                                    grabber.grabberIn(),
                                                    arm.liftToTargetPosition(0)
                                            ),
                                            new ParallelAction(
                                                    intake.intakeIn(),
                                                    grabber.grabberRelease()
                                            )
                                    )
                            ),
                            grabber.intakeUp(),
                            grabber.grabberGrab()
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
                            scoreThirdSample.build(),
                            new SequentialAction(
                                    grabber.intakeDown(),
                                    arm.liftToTargetPosition(Arm.LIFT_BASKET),
                                    grabber.grabberOut()
                            )
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberRelease(),
                            driveToPark.build()
                    )
            ); */


        }
    }}