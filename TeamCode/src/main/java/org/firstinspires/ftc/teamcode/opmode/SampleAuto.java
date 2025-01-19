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
@Autonomous(name = "Sample Auto", group = "Autonomous")
public class SampleAuto extends DuckbotAuto {

    private MecanumDrive drive = null;
    private Grabber teleGrabber = null;
    private AutoArm arm = null;
    private AutoIntake intake = null;
    private AutoGrabber grabber = null;

    private static final TranslationalVelConstraint SLOW_VEL = new TranslationalVelConstraint(15.0);
    private static final ProfileAccelConstraint SLOW_ACC = new ProfileAccelConstraint(-10, 10);

    private TrajectoryActionBuilder cycleSample(
            TrajectoryActionBuilder start,
            double sampleX, double sampleY, double sampleHeading,
            double scoreX, double scoreY, double scoreHeading){

        TrajectoryActionBuilder driveToSample = buildSimpleTrajectory(start, sampleX, sampleY, sampleHeading);
        TrajectoryActionBuilder driveToScore = buildSimpleTrajectory(driveToSample, scoreX, scoreY, scoreHeading);

        /* Pick up Sample */
        Actions.runBlocking(
                new SequentialAction(
                        grabber.grabberIn(),
                        new ParallelAction(
                                arm.liftToTargetPosition(0),
                                intake.intakeDown()
                        ),
                        new ParallelAction(
                                driveToSample.build() // will need to alter the grab pos to be more accurate
                        ),
                        intake.intakeClose(),
                        intake.intakeUp(),
                        intake.retract(),
                        new SleepAction(1.0),
                        grabber.grabberGrab(),
                        new SleepAction(0.5)
                )
        );

        /* Score Sample */
        Actions.runBlocking(
                new ParallelAction(
                        driveToScore.build(),
                        new SequentialAction(
                                intake.intakeOpen(), //might need wait in between
                                intake.intakeDown(),
                                arm.liftToTargetPosition(Arm.LIFT_BASKET),
                                grabber.grabberOut(),
                                new SleepAction(1.0),
                                grabber.grabberRelease(),
                                new SleepAction(0.5)
                        )
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

        teleGrabber = hardwareStore.getGrabber();
        drive = hardwareStore.getDrive();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        teleGrabber.closeGrabber();

        waitForStart();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        Vector2d scoringPose = new Vector2d(8, 17);
        Vector2d closeScoringPose = new Vector2d(14, 26); //arbitrary value
        double scoringHeading = Math.toRadians(-45);

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
                            new SleepAction(0.5),
                            grabber.grabberOut(),
                            new SleepAction(0.5),
                            grabber.grabberRelease(),
                            new SleepAction(0.5)
                    )

            );

            /* Cycle 3 samples */
            TrajectoryActionBuilder cycle1 = cycleSample(scorePreload,
                    31, 8.8, 0,
                    8, 17, 0);

            TrajectoryActionBuilder cycle2 = cycleSample(cycle1,
                    31, 17.5, 0,
                    8, 17, 0);

            TrajectoryActionBuilder cycle3 = cycleSample(cycle2,
                    -26, -4, 0,
                    8, 17, 0);

            /* Park */
            TrajectoryActionBuilder driveToPark = cycle3.endTrajectory().fresh()
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
            );
        }
    }}