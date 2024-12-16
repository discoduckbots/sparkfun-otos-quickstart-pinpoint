package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;

public abstract class DuckbotAuto extends LinearOpMode {

    private static final double INTAKE_TIME = 1.5;
    private static final double INTAKE_SPEED = 1.0;

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

                if (getRuntime()-startTime < INTAKE_TIME){
                    return true;
                }

                intake.stop();
                return  false;
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

        public Action intakeIn(){
            return new IntakeIn();
        }

        public Action intakeOut(){
            return  new IntakeOut();
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

        public Action intakeUp(){
            return new IntakeUp();
        }

        public Action intakeDown(){
            return new IntakeDown();
        }
    }
}
