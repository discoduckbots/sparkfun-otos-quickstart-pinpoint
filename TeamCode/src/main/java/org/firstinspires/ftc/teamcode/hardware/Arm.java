package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {
    public DcMotor liftLeft;
    public DcMotor liftRight;


    public Arm(DcMotor liftMotor1, DcMotor liftMotor2) {

        this.liftLeft = liftMotor1;
        this.liftLeft.setDirection(DcMotor.Direction.FORWARD);
        this.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.liftRight = liftMotor2;
        this.liftRight.setDirection(DcMotor.Direction.FORWARD);
        this.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getLiftPos() {
        return liftLeft.getCurrentPosition();
    }

    public int getLiftPos2() {
        return liftRight.getCurrentPosition();
    }

    public void lift(double power) {
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(power);
        liftRight.setPower(power);
        Log.d("LIFT ", "pos1: " + liftLeft.getCurrentPosition() + "pos2: " + liftRight.getCurrentPosition());
    }

    public void lower(double power) {
        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setPower(power);
        liftRight.setPower(power);
        Log.d("LIFT ", "pos1: " + liftLeft.getCurrentPosition() + "pos2: " + liftRight.getCurrentPosition());
    }

    public void stop(){
        liftLeft.setPower(0);
        liftRight.setPower(0);
    }


    public void liftToPosition(int position, int position2, double power) {
        liftLeft.setTargetPosition(position);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setTargetPosition(position2);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

}
