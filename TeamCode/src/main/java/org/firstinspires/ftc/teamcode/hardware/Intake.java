package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public CRServo intakeLeft;
    public CRServo intakeRight;
    public DcMotor extensionMotor;

    public Intake(CRServo intakeLeft, CRServo intakeRight, DcMotor extensionMotor) {
        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
        this.extensionMotor = extensionMotor;
    }

    public void intake(double power) {
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setPower(power);
        intakeRight.setPower(power);

    }

    public void outtake(double power) {
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void stop(){
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    public void extendStop(){
        extensionMotor.setPower(0);
    }

    public void extend(double power){
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setPower(power);
    }

    public void retract(double power){
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotor.setPower(power);
    }

}
