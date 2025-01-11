package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.ScoringMechanism;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Duckbot Op Mode", group= "Linear Opmode")
public class DuckbotTeleop extends LinearOpMode {

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
        grabber.flipIntakeDown();

        boolean inGrabPosition = false;

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
                grabber.flipIntakeDown();
            }
            if (gamepad1.y) {
                grabber.flipIntakeUp();
            }

            if (gamepad2.dpad_up){
                arm.lift(LIFT_SPEED);
            }
            else if (gamepad2.dpad_down){
                arm.lower(LOWER_SPEED);
            }
            else{
                arm.liftByEncoder(arm.getLiftPos(), LOWER_SPEED);
            }

            if (gamepad2.left_trigger > 0.25) {
                arm.liftByEncoder(Arm.LIFT_ABOVE_BAR, LIFT_SPEED);
            }

            if (gamepad2.right_trigger > 0.25) {
                arm.liftByEncoder(Arm.LIFT_BASKET, LIFT_SPEED);
            }

            if (gamepad2.left_bumper){
                grabber.grab();
            }
            else if (gamepad2.right_bumper){
                grabber.release();
            }

            if (gamepad2.a) {
                grabber.flipGrabberOut();
            }
            else if (gamepad2.b) {
                grabber.flipGrabberIn();
            }

            /*if(gamepad2.x) {
                grabber.flipIntakeDown();
            }
            else if (gamepad2.y) {
                grabber.flipIntakeUp();
            } */

            if (gamepad2.y) {
                grabber.flipIntakeDown();
            }

            /*if (gamepad2.x) {
                grabber.flipIntakeUp();
            } */

            if (gamepad2.x && !inGrabPosition) {
                inGrabPosition = true;
                //intake.extendByEncoder(EXTENSION_SPEED);
                grabber.flipGrabberIn();
                grabber.release();
                grabber.flipIntakeUp();
                arm.liftByEncoder(Arm.LIFT_MIN, LIFT_SPEED);
                sleep(250);
                intake.retractByEncoder(EXTENSION_SPEED);
                sleep(500);
                grabber.grab();
                sleep(250);
                grabber.flipIntakeDown();
//                scoringMechanism.grabFromIntake(EXTENSION_SPEED, ARM_SPEED, this);
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

}