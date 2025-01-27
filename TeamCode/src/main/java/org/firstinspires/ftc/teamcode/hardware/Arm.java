package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {
    public DcMotor liftLeft;
    public DcMotor liftRight;
    public TouchSensor leftLimitSwitch;
    public TouchSensor rightLimitSwitch;

    public static final int LIFT_MAX = 4670;
    public static final int LIFT_BASKET = 4200;
    public static final int LIFT_ABOVE_BAR = 1750;
    public static final int LIFT_PLACE_SPECIMEN = 1950;
    public static final int LIFT_BELOW_BAR = 1600;
    public static final int LIFT_GRAB_FROM_WALL = 120;
    public static final int LIFT_RAISE_ABOVE_WALL = 150;
    public static final int LIFT_TOUCH_LOW_BAR = 1250;
    public static final int LIFT_ABOVE_LOW_BAR = 1400;
    public static final int LIFT_MIN = 0;

    // high goal - L: -4418, R: -6302
// above bar - L: 2134, R:4154
    // below bar - L-1193, R: -3206

    public Arm(DcMotor liftMotor1, DcMotor liftMotor2, TouchSensor leftLimitSwitch, TouchSensor rightLimitSwitch) {

        this.liftLeft = liftMotor1;
        this.liftLeft.setDirection(DcMotor.Direction.FORWARD);
        this.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.liftRight = liftMotor2;
        this.liftRight.setDirection(DcMotor.Direction.REVERSE);
        this.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;
    }

    public int getLiftPos() {
        return liftLeft.getCurrentPosition();
    }

    public int getLiftPos2() {
        return liftRight.getCurrentPosition();
    }

    public void lift(double power) {
        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(power);
        liftRight.setPower(power);
        Log.d("LIFT ", "pos1: " + liftLeft.getCurrentPosition() + "pos2: " + liftRight.getCurrentPosition());

    }

    public void lower(double power) {
        if (!leftLimitSwitch.isPressed() && !rightLimitSwitch.isPressed()) {
            liftLeft.setDirection(DcMotor.Direction.REVERSE);
            liftRight.setDirection(DcMotor.Direction.FORWARD);
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftLeft.setPower(power);
            liftRight.setPower(power);
            Log.d("LIFT ", "pos1: " + liftLeft.getCurrentPosition() + "pos2: " + liftRight.getCurrentPosition());
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stop() {
        liftLeft.setPower(0);
        liftRight.setPower(0);
    }

    public void liftByEncoder(int position, double power) {

        liftLeft.setTargetPosition(position);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setTargetPosition(position);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

    public void lowerByTouch(double power) {
        stop();
        while (!leftLimitSwitch.isPressed()) {
            lower(power);
        }
    }

}
