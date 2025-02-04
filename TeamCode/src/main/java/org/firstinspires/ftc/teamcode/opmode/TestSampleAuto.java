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
import org.firstinspires.ftc.teamcode.hardware.Intake;

@Config
@Autonomous(name = "Test Sample Auto", group = "Autonomous")
public class TestSampleAuto extends DuckbotAuto {

    private Grabber teleGrabber = null;
    private Intake teleIntake = null;
    private AutoArm arm = null;
    private AutoIntake intake = null;
    private AutoGrabber grabber = null;
    MecanumDrive drive = null;

    private static final TranslationalVelConstraint SLOW_VEL = new TranslationalVelConstraint(15.0);
    private static final ProfileAccelConstraint SLOW_ACC = new ProfileAccelConstraint(-10, 10);

    private TrajectoryActionBuilder cycleSample( //This is a method that is called to cycle a sample
            TrajectoryActionBuilder start, double rotatePos,
            double sampleX, double sampleY, double sampleHeading,
            double scoreX, double scoreY, double scoreHeading) {

        TrajectoryActionBuilder driveToSample = buildSimpleTrajectory(start, sampleX-10, sampleY, sampleHeading); //changed so it goes a little less
        TrajectoryActionBuilder driveForwardLittle = driveToSample.endTrajectory().fresh()
                .strafeTo(new Vector2d(sampleX, sampleY));
        TrajectoryActionBuilder driveToScore = buildSimpleTrajectory(driveForwardLittle, scoreX, scoreY, scoreHeading);

        /* Pick up Sample */
        Actions.runBlocking(
                new SequentialAction(
                        grabber.grabberGrab(),
                        new SleepAction(0.5),
                        grabber.grabberIn(),
                        new SleepAction(0.2),
                        new ParallelAction(
                                arm.liftToTargetPosition(Arm.LIFT_GRAB_FROM_WALL),
                                grabber.grabberRelease(),
                                intake.intakeOpen(),
                                driveToSample.build()
                        ),
                        new ParallelAction(
                                arm.liftToTargetPosition(0),
                                driveForwardLittle.build(), //I added a driveForwardLittle Traj to prevent hitting the blocks
                                //intake.intakeDown() old code
                                new SequentialAction( // new change
                                        intake.intakeDown(),
                                        new SleepAction(0.2),
                                        intake.intakeRotate(rotatePos)
                                )
                                ),
                        new SleepAction(1.0),
                        intake.intakeClose(),
                        new SleepAction(0.4),
                        intake.intakeRotate(0), //new change
                        new SleepAction(0.3), //new change
                        intake.intakeUp(),
                        intake.retract(),
                        new SleepAction(0.4),
                        grabber.grabberGrab(),
                        new SleepAction(0.2)
                )
        );


        /* Score Sample */
        Actions.runBlocking(
                new SequentialAction(
                    new ParallelAction(
                            driveToScore.build(),
                            new SequentialAction(
                                    intake.intakeOpen(),
                                    new SleepAction(0.2),
                                    intake.intakeDown(),
                                    arm.liftToTargetPosition(Arm.LIFT_BASKET)
                            )
                    ),
                    new SleepAction(1.2),
                    grabber.grabberOut(),
                    new SleepAction(1.0),
                    grabber.grabberRelease(),
                    new SleepAction(1.0)
                )
        );

        return driveToScore;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        arm = new AutoArm(hardwareStore);
        intake = new AutoIntake(hardwareStore);
        grabber = new AutoGrabber(hardwareStore);

        drive = hardwareStore.getDrive();

        teleGrabber = hardwareStore.getGrabber();
        teleIntake = hardwareStore.getIntake();
        MecanumDrive drive = hardwareStore.getDrive();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        grabber.grabberGrab(),
                        grabber.grabberMiddle()
                )
        );

        teleIntake.rotateIntakeTo0();

        waitForStart();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Vector2d scoringPose = new Vector2d(11.2, 10.8);
        double scoringHeading = -45;

        if (opModeIsActive()) {
            if (isStopRequested()) return;

            /* Score Preload Sample */
            TrajectoryActionBuilder scorePreload = drive.actionBuilder(initialPose)
                    .strafeToLinearHeading(scoringPose, scoringHeading); //strafe to score preload

            Actions.runBlocking(
                    new SequentialAction(
                            grabber.grabberGrab(),
                            new ParallelAction(
                                    scorePreload.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_BASKET)
                                    ),
                            new SleepAction(1.0),
                            grabber.grabberOut()
                            /*
                            new SleepAction(1.0),
                            grabber.grabberRelease(),
                            new SleepAction(1.0) */
                    )

            );

            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(1.0),
                            grabber.grabberRelease(),
                            new SleepAction(1.0)
                    )
            );

            /* Cycle 3 samples - This is where the actually positions are inputted*/
            TrajectoryActionBuilder cycle1 = cycleSample(scorePreload, 0,
                    29, 6.5, 0,
                    scoringPose.x, scoringPose.y, scoringHeading); //position found was 8.5 / 12

            TrajectoryActionBuilder cycle2 = cycleSample(cycle1, 0,
                    29, 17.5, 0,
                    scoringPose.x, scoringPose.y, scoringHeading);
// need to test
            TrajectoryActionBuilder cycle3 = cycleSample(cycle2, 1.0,// I don't know if the 3rd one is going to work b/c we probably need to rotate the intake to grab it
                   -30, -18, 90,
                    scoringPose.x, scoringPose.y, scoringHeading);

            /* Park */ /*
            TrajectoryActionBuilder driveToPark = cycle2.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(44, -7), Math.toRadians(90)) // back up from basket and turn
                    .strafeTo(new Vector2d(44, -23.5)); // strafe to chamber //linear slide touch bar pos: 1250

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    driveToPark.build(),
                                    arm.liftToTargetPosition(Arm.LIFT_ABOVE_LOW_BAR)
                            ),
                            arm.liftToTargetPosition(Arm.LIFT_TOUCH_LOW_BAR)
                    )
            ); */
        }
    }}