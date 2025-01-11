package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private static final int EXTEND_OUT = 430;
    private static final int EXTEND_LITTLE = 100;
    private static final int EXTEND_IN = 0;
    public CRServo intakeLeft;
    public CRServo intakeRight;
    public DcMotor extensionMotor;


    public Intake(CRServo intakeLeft, CRServo intakeRight, DcMotor extensionMotor) {
        this.intakeLeft = intakeLeft;
        this.intakeRight = intakeRight;
        this.extensionMotor = extensionMotor;
        this.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public int getExtensionPos() {
        return extensionMotor.getCurrentPosition();
    }
    public void extendStop(){
        extensionMotor.setPower(0);
    }

    public void extend(double power){
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setPower(power);
    }

    public void retract(double power){
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotor.setPower(power);
    }

    public void extendByEncoder(double power){
        extensionMotor.setTargetPosition(EXTEND_OUT);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }

    public void extendLittleByEncoder(double power){
        extensionMotor.setTargetPosition(EXTEND_LITTLE);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }

    public void retractByEncoder(double power){
        extensionMotor.setTargetPosition(EXTEND_IN);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }

    public void holdPosition(int position, double power){
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }
}
