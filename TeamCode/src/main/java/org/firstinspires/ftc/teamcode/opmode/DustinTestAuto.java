package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;

@Config
@Autonomous(name = "DUSTIN_TEST_AUTO", group = "Autonomous")
public class DustinTestAuto extends DuckbotAuto{

    MecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        AutoIntake intake = new AutoIntake(hardwareStore);
        AutoGrabber grabber = new AutoGrabber(hardwareStore);

        drive = hardwareStore.getDrive();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(20);


        if (opModeIsActive()) {
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build(),
                            intake.intakeIn(),
                            grabber.grabberGrab(),
                            intake.intakeOut(),
                            grabber.grabberRelease()
                    )
            );
        }
    }

}
