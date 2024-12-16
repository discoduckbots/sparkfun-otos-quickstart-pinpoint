package org.firstinspires.ftc.teamcode.hardware;

public class ScoringMechanism {
    Arm arm = null;
    Grabber grabber = null;
    Intake intake = null;

    public ScoringMechanism(Arm arm, Grabber grabber, Intake intake) {
        this.arm = arm;
        this.grabber = grabber;
        this.intake = intake;
    }

    public void retract(double retractPower) {
        intake.retractByEncoder(retractPower);
        grabber.flipGrabberIn();
    }
}
