package org.firstinspires.ftc.teamcode.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.ScoringMechanism;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Fancy Cancelable Teleop", group= "Linear Opmode")
public class FancyCancelableTeleop extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;
    Arm arm = null;
    Intake intake = null;
    Grabber grabber = null;
    TouchSensor leftLimitSwitch = null;
    ScoringMechanism scoringMechanism = null;
    PinpointDrive drive = null;

    private double THROTTLE = 0.7;
    private double TURN_THROTTLE = 0.5;
    private double LIFT_SPEED = 1.0;
    private double LOWER_SPEED = 0.7;
    private double EXTENSION_SPEED = 0.7;
    private boolean inGrabPosition = false;

    private Pose2d lastPositionA = null;
    private Pose2d lastPositionB = null;
    boolean lastPositionPressed = false;
    Action moveToLastPosAction = null;



    @Override
    public void runOpMode() throws InterruptedException {
        int extendPosition = 0;

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        arm = hardwareStore.getArm();
        intake = hardwareStore.getIntake();
        grabber = hardwareStore.getGrabber();
        scoringMechanism = hardwareStore.getScoringMechanism();
        drive = hardwareStore.getDrive();
        leftLimitSwitch = hardwareStore.leftLimitSwitch;

        //grabber.grab();

        waitForStart();
        //intake.flipIntakeDown();
        intake.rotateIntakeTo0();

        boolean inGrabPosition = false;

        while (opModeIsActive()) {
            driveControl(drive);

            if (gamepad1.right_trigger > 0.1){
                intake.openIntake();
            }

            if (gamepad1.left_trigger > 0.1){
                intake.closeIntake();
            }

            if (gamepad1.right_bumper){
                intake.extend(EXTENSION_SPEED);
            }
            else if (gamepad1.left_bumper){
                intake.retractByEncoder(EXTENSION_SPEED);
            }
            else {
                intake.extendStop();
            }


            if (gamepad1.x) {
                intake.flipIntakeDown();
                telemetry.addData("trying to flip down", intake.intakeFlip.getPosition());
                telemetry.update();
            }
            if (gamepad1.y) {
                intake.flipIntakeUp();
                telemetry.addData("trying to flip up", intake.intakeFlip.getPosition());
                telemetry.update();
            }

            /*if (gamepad1.a) {
                intake.rotateIntakeTo0();
            } */
            if (gamepad1.b) {
                intake.rotateIntakeTo90();
            }
            if (gamepad2.right_stick_y > 0.05) {
                intake.extend(gamepad2.right_stick_y);
            }
            else if (gamepad2.right_stick_y < 0.05) {
                intake.extend(gamepad2.right_stick_y);
            }

            if (gamepad2.dpad_up){
                arm.lift(LIFT_SPEED);
            }
            else if (gamepad2.dpad_down){
                arm.lower(LIFT_SPEED);
            }
            else{
                arm.stop();
            }

            if (gamepad2.left_trigger > 0.2) {
                intake.onPressFlip();
            }
            else {
                intake.onReleaseFlip();
            }

            if (gamepad2.left_bumper) {
                intake.onPressIntake();
            }
            else {
                intake.onReleaseIntake();
            }

            if (gamepad2.right_bumper) {
                grabber.onPressGrabber();
            }
            else {
                grabber.onReleaseGrabber();
            }


            if (gamepad2.a) {
                grabber.onPressFlip();
            }
            else {
                grabber.onReleaseFlip();
            }

            if (gamepad2.x && !inGrabPosition) {
                inGrabPosition = true;
                grabber.flipGrabberIn();
                grabber.openGrabber();
                intake.flipIntakeUp();
                arm.liftByEncoder(0, LIFT_SPEED);
                intake.retractByEncoder(EXTENSION_SPEED);
                sleep(200);
                grabber.closeGrabber();
                sleep(200);
                intake.openIntake();
                intake.flipIntakeDown();
                inGrabPosition = false;
            }




            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("lift_left ", arm.liftLeft.getCurrentPosition());
            telemetry.addData("lift_right ", arm.liftRight.getCurrentPosition());
            telemetry.addData("extension ", intake.extensionMotor.getCurrentPosition());
            telemetry.addData("l_limit_switch ", leftLimitSwitch.getValue());
            telemetry.update();

        }

        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");
    }

    private void driveControl(PinpointDrive drive)
    {
        Pose2d poseEstimate = drive.getPose();
        drive.updatePoseEstimate();
        Log.d("LOC", "x = " + poseEstimate.position.x +
                " y= " + poseEstimate.position.y +
                " heading " + Math.toDegrees(poseEstimate.heading.real));
        switch (currentMode) {
            case DRIVER_CONTROL:
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * THROTTLE,
                                -gamepad1.left_stick_x * THROTTLE
                        ),
                        -gamepad1.right_stick_x * TURN_THROTTLE
                ));

                drive.updatePoseEstimate();
                if (gamepad1.dpad_up) {
                    lastPositionA = poseEstimate;
                    Log.d("LAST", "Setting last position to " + lastPositionA);
                }
                if (gamepad1.dpad_left) {
                    lastPositionB = poseEstimate;
                    Log.d("LAST", "Setting last position to " + lastPositionB);
                }
                if (gamepad1.dpad_down || gamepad1.dpad_right) {
                    if (!lastPositionPressed) {
                        lastPositionPressed = true;
                        Pose2d lastPosition = null;
                        if (gamepad1.dpad_down) lastPosition = lastPositionA;
                        else lastPosition = lastPositionB;
                        if (lastPosition != null) {
                            currentMode = Mode.AUTOMATIC_CONTROL;
                            TrajectoryActionBuilder moveToLastPosition = drive.actionBuilder(drive.getPose())
                                    .strafeToLinearHeading(new Vector2d(lastPosition.position.x, lastPosition.position.y), lastPosition.heading);
                            moveToLastPosAction = moveToLastPosition.build();

                        }
                    }
                }
                else {
                    lastPositionPressed = false;
                }

                break;
            case AUTOMATIC_CONTROL:
                if (gamepad1.a) {
                    //drive.
                    currentMode = Mode.DRIVER_CONTROL;
                    return;
                }
               if (moveToLastPosAction != null) {
                   TelemetryPacket packet = new TelemetryPacket();
                   moveToLastPosAction.preview(packet.fieldOverlay());
                   if (!moveToLastPosAction.run(packet)) {
                       currentMode = Mode.DRIVER_CONTROL;
                   }

               }
        }
    }

}