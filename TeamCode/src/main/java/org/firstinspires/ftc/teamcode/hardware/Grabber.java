package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    Servo grabberServo = null;
    Servo grabberFlip = null;
    Servo intakeFlip = null;

    private boolean intakeUp = false;
    private boolean grabberIn = true;
    private boolean grabberOpen = false;


    public Grabber(Servo grabberServo, Servo grabberFlip, Servo intakeFlip){

        this.grabberServo = grabberServo;
        this.grabberFlip = grabberFlip;
        this.intakeFlip = intakeFlip;
    }

    public void grab(){
        grabberServo.setPosition(1.0);
        grabberOpen = false;
    }

    public void release(){
        grabberServo.setPosition(0.5);
        grabberOpen = true;
    }

    public void flipGrabberOut(){
        if (grabberIn) {
            if (grabberOpen) {
                grab();
            }
            if (!grabberOpen) {
                grabberFlip.setDirection(Servo.Direction.REVERSE);
                //only if linear slide is all the way down
                grabberFlip.setPosition(0.0);
                grabberIn = false;
            }
        }
    }

    public void flipGrabberIn(){
        if (!grabberIn) {
            if (grabberOpen) {
                grab();
            }
            if (!grabberOpen) {
                grabberFlip.setDirection(Servo.Direction.REVERSE); //test comment
                grabberFlip.setPosition(1.0);
                grabberIn = true;
            }
        }
    }

    public void flipIntakeUp() {
        if (!grabberIn) {
            flipGrabberIn();
        }
        if (!grabberOpen) {
            release();
        }
        intakeFlip.setPosition(1.0);
        intakeUp = true;
    }

    public void flipIntakeDown() {
        intakeFlip.setPosition(0.0);
        intakeUp = false;
    }

}
