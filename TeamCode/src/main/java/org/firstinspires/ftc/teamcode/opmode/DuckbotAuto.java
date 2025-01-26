package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;
import org.firstinspires.ftc.teamcode.hardware.Intake;

public abstract class DuckbotAuto extends LinearOpMode {

    private static final double INTAKE_TIME = 1.5;
    private static final double INTAKE_SPEED = 1.0;
    private static final double LIFT_SPEED = 1.0;
    private static final double EXTENSION_SPEED = 0.3;

    /**
     * Given a starting position and a desired x, y, heading, returns a trajectory
     * that strafes to the desired position
     *
     * @param start - Starting Position
     * @param x     - Desired X Position
     * @param y     - Desired Y Position
     * @param heading - Desired Heading
     * @return TrajectoryActionBuilder
     */
    protected TrajectoryActionBuilder buildSimpleTrajectory (TrajectoryActionBuilder start, double x, double y, double heading) {
        return start.fresh()
                .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading));
    }

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

        public class IntakeClose implements Action {
            private boolean initialized = false;
            private double startTime = getRuntime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    intake.closeIntake();
                }

                /*if (getRuntime()-startTime < INTAKE_TIME){
                    return true;
                }

                intake.stop(); */
                return  false;
            }
        }

        public class IntakeOpen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    intake.openIntake();
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
                    intake.flipIntakeUp();
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
                    intake.flipIntakeDown();
                }
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
                    intake.extendByEncoder(EXTENSION_SPEED);
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


        public Action intakeClose(){
            return new IntakeClose();
        }


        public Action intakeOpen(){
            return new IntakeOpen();
        }

        public Action intakeUp(){
            return new IntakeUp();
        }

        public Action intakeDown(){
            return new IntakeDown();
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
                    grabber.closeGrabber();
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
                    grabber.openGrabber();
                }

                return false;
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

    }
}
