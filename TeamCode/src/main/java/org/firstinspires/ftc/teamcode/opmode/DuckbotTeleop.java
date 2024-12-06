package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Duckbot Op Mode", group= "Linear Opmode")
public class DuckbotTeleop extends LinearOpMode {

    Arm arm = null;
    Intake intake = null;
    Grabber grabber = null;
    PinpointDrive drive = null;

    private double THROTTLE = 0.7;
    private double TURN_THROTTLE = 0.5;
    private double ARM_SPEED = 0.5;
    private double EXTENSION_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        arm = hardwareStore.getArm();
        intake = hardwareStore.getIntake();
        grabber = hardwareStore.getGrabber();
        drive = hardwareStore.getDrive();
        //grabber = hardwareStore.getGrabber();

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * THROTTLE,
                            -gamepad1.left_stick_x * THROTTLE
                    ),
                    -gamepad1.right_stick_x * TURN_THROTTLE
            ));

            drive.updatePoseEstimate();

            if (gamepad1.right_trigger > 0.25){
                intake.intake(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger > 0.25){
                intake.outtake(gamepad1.left_trigger);
            }
            else{
                intake.stop();
            }

            if (gamepad1.b){
                intake.extend(EXTENSION_SPEED);
            }
            else if (gamepad1.a){
                intake.retract(EXTENSION_SPEED);
            }
            else{
                intake.extendStop();
            }

            if (gamepad2.dpad_up){
                arm.lift(ARM_SPEED);
            }
            else if (gamepad2.dpad_down){
                arm.lower(ARM_SPEED);
            }
            else{
                arm.stop();
            }

            if (gamepad2.left_trigger > 0.25) {
                ARM_SPEED = 1.0;
            }

            if (gamepad2.right_trigger > 0.25) {
                ARM_SPEED = 0.5;
            }

            if (gamepad2.left_bumper){
                grabber.grab();
            }
            else if (gamepad2.right_bumper){
                grabber.release();
            }

            if (gamepad2.a) {
                grabber.flipGrabberOut(); //Out is in pos
            }
            else if (gamepad2.b) {
                grabber.flipGrabberIn();
            }

            if(gamepad2.x) {
                grabber.flipIntakeDown();
            }
            else if (gamepad2.y) {
                grabber.flipIntakeUp();
            }

            if(gamepad1.x) {
                grabber.fancyGrab();
            }


            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

        }

        telemetry.addData("MecanumDrivetrainTeleOp", "Stopping");
    }

}