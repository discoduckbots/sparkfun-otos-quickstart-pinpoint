package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;

public abstract class DuckbotAuto extends LinearOpMode {

    private static final double INTAKE_TIME = 1.5;
    private static final double INTAKE_SPEED = 1.0;
    private static final double LIFT_SPEED = 1.0;
    private static final double EXTENSION_SPEED = 0.3;
    public class AutoArm {
        private Arm arm;

        public AutoArm(HardwareStore hardwareStore){
            arm = hardwareStore.getArm();
        }

        public class ArmLiftToPosition implements Action {


            private boolean initialized = false;
            private int targetPosition = 0;

            public ArmLiftToPosition(int targetPosition){
                super();
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    arm.liftByEncoder(targetPosition, LIFT_SPEED);
                    initialized = true;
                }

                if (arm.getLiftPos() != targetPosition){
                    return true;
                }

                return false;
            }
        }

        public class LowerToZero implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    resetRuntime();
                    arm.lowerByTouch(LIFT_SPEED);
                }

                return false;
            }
        }


        public Action liftToTargetPosition(int targetPosition){
            return new ArmLiftToPosition(targetPosition);
        }

        public Action lowerToZero(){
            return new LowerToZero();
        }
    }

    public class AutoIntake {

        private Intake intake;

        public AutoIntake(HardwareStore hardwareStore){
            intake = hardwareStore.getIntake();
        }

        public class IntakeIn implements Action {
            private boolean initialized = false;
            private double startTime = getRuntime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    intake.intake(INTAKE_SPEED);
                }

                /*if (getRuntime()-startTime < INTAKE_TIME){
                    return true;
                }

                intake.stop(); */
                return  false;
            }
        }

        public class StopIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    intake.stop();
                }
                return false;
            }
        }

        public class IntakeOut implements Action {
            private boolean initialized = false;
            private double startTime = getRuntime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    intake.outtake(INTAKE_SPEED);
                }

                if (getRuntime()-startTime < INTAKE_TIME){
                    return true;
                }

                intake.stop();
                return  false;
            }
        }

        public class Extend implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    resetRuntime();
                    intake.extendLittleByEncoder(EXTENSION_SPEED);
                }

                return false;
            }
        }

        public class Retract implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    resetRuntime();
                    intake.retractByEncoder(EXTENSION_SPEED);
                }

                return false;
            }
        }


        public Action intakeIn(){
            return new IntakeIn();
        }

        public Action intakeOut(){
            return  new IntakeOut();
        }

        public Action stopIntake(){
            return new StopIntake();
        }

        public Action extend(){
            return new Extend();
        }

        public Action retract(){
            return new Retract();
        }
    }

    public class AutoGrabber {
        private Grabber grabber;

        public AutoGrabber(HardwareStore hardwareStore){
            grabber = hardwareStore.getGrabber();
        }

        public class GrabberGrab implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    resetRuntime();
                    grabber.grab();
                }

                return false;
            }
        }

        public class GrabberRelease implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    resetRuntime();
                    grabber.release();
                }

                return false;
            }
        }

        public class IntakeUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    grabber.flipIntakeUp();
                }
                return  false;
            }
        }

        public class IntakeDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    grabber.flipIntakeDown();
                }
                return  false;
            }
        }

        public class GrabberIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    grabber.flipGrabberIn();
                }
                return  false;
            }
        }

        public class GrabberOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    grabber.flipGrabberOut();
                }
                return  false;
            }
        }

        public class GrabberMiddle implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    grabber.flipGrabberMiddle();
                }
                return  false;
            }
        }

        public Action grabberGrab(){
            return new GrabberGrab();
        }

        public Action grabberRelease(){
            return new GrabberRelease();
        }

        public Action grabberIn(){
            return new GrabberIn();
        }

        public Action grabberOut(){
            return new GrabberOut();
        }

        public Action grabberMiddle(){
            return new GrabberMiddle();
        }

        public Action intakeUp(){
            return new IntakeUp();
        }

        public Action intakeDown(){
            return new IntakeDown();
        }
    }
}
