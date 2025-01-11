package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    Servo grabberServo = null;
    Servo grabberFlip = null;
    Servo intakeFlip = null;

    private boolean intakeUp = false;
    private boolean grabberIn;
    private boolean grabberOut;
    private boolean grabberMid;
    private boolean grabberOpen = false;


    public Grabber(Servo grabberServo, Servo grabberFlip, Servo intakeFlip){

        this.grabberServo = grabberServo;
        this.grabberFlip = grabberFlip;
        this.intakeFlip = intakeFlip;
    }

    public void grab(){
        grabberServo.setPosition(0.5);
        grabberOpen = false;
    }

    public void release(){
        grabberServo.setPosition(0);
        grabberOpen = true;
    }

    public boolean isGrabberOpen() {
        return grabberOpen;
    }

    public void flipGrabberOut(){
        if (grabberOpen) {
            grab();
        }
        if (!grabberOpen) {
            grabberFlip.setDirection(Servo.Direction.REVERSE);
            //only if linear slide is all the way down
            grabberFlip.setPosition(0.0);
            grabberOut = true;
            grabberIn = false;
            grabberMid = false;
        }
    }

    public void flipGrabberIn(){
        if (grabberOpen) {
            grab();
        }
        if (!grabberOpen) {
            grabberFlip.setDirection(Servo.Direction.REVERSE); //test comment
            grabberFlip.setPosition(1.0);
            grabberIn = true;
            grabberOut = false;
            grabberMid = false;

        }
    }


    public void flipGrabberMiddle(){
        grabberFlip.setPosition(0.5);
        grabberMid = true;
        grabberOut = false;
        grabberIn = false;
    }

    public boolean isGrabberIn() {
        return grabberIn;
    }

    public boolean isGrabberOut() {
        return grabberOut;
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

    public boolean isIntakeUp() {
        return intakeUp;
    }

}
