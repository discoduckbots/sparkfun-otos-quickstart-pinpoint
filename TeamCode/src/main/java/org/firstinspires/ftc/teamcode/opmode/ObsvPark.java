package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;

@Disabled
@Autonomous(name = "obsvPark", group = "Robot")
public class ObsvPark extends LinearOpMode {
    Arm arm = null;
    Grabber grabber = null;
    MecanumDrive drive = null;

    @Override
    public void runOpMode() {
        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        arm = hardwareStore.getArm();
        drive = hardwareStore.getDrive();
//        grabber = hardwareStore.getGrabber();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Pose2d startPose = new Pose2d(0, 0, 0);
        Vector2d firstMove = new Vector2d(40, 0);

        waitForStart();

        if (opModeIsActive()) {

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .strafeTo(firstMove, null, null)
                            .build());
        }
    }
}