package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ScoringMechanism {
    Arm arm = null;
    Grabber grabber = null;
    Intake intake = null;

    public ScoringMechanism(Arm arm, Grabber grabber, Intake intake) {
        this.arm = arm;
        this.grabber = grabber;
        this.intake = intake;
    }
/*
    private void retractAndGrabberIn(double retractPower) {
        grabber.flipGrabberIn();
        intake.retractByEncoder(retractPower);

    }

    private void lowerAndOpen(double liftPower, double retractPower) {
        if (!grabber.isGrabberIn() || intake.getExtensionPos() > 1) {
            retractAndGrabberIn(retractPower);
        }
        if (grabber.isGrabberIn() && intake.getExtensionPos() < 1) {
            arm.liftByEncoder(0, liftPower);
            grabber.openGrabber();
        }
    }
    private void linkedIntakeUp(double liftPower, double retractPower) {
        if (arm.getLiftPos() > 3) {
            lowerAndOpen(liftPower, retractPower);
        }
        if (arm.getLiftPos() < 3) {
            grabber.flipIntakeUp();
        }
    }

    private void linkedCloseGrabber(double liftPower, double retractPower) {
        if (!grabber.isIntakeUp()) {
            linkedIntakeUp(liftPower, retractPower);
        }
        if (grabber.isIntakeUp()) {
            grabber.closeGrabber();
        }
    }

    private void linkedIntakeDown(double liftPower, double retractPower) {
        if (grabber.isGrabberOpen()) {
            linkedCloseGrabber(liftPower, retractPower);
        }
        if (!grabber.isGrabberOpen()) {
            grabber.flipIntakeDown();
        }
    }

    public void scoreSample(double liftPower, double retractPower) {
        retractAndGrabberIn(retractPower);
        lowerAndOpen(liftPower, retractPower);
        linkedIntakeUp(liftPower, retractPower);
        linkedCloseGrabber(liftPower, retractPower);
        linkedIntakeDown(liftPower, retractPower);
    }

    /**
     * Preconditions:
     *      A Block is Loaded in the Intake
     */
    public void grabFromIntake(double retractPower, double liftPower, OpMode opMode){
        grabber.flipGrabberIn();
//        grabber.release();
//        arm.liftByEncoder(Arm.LIFT_MIN, liftPower);
//        grabber.flipIntakeUp();
//        intake.retractByEncoder(retractPower);
//        grabber.grab();
//        grabber.flipIntakeDown();

    }
}



