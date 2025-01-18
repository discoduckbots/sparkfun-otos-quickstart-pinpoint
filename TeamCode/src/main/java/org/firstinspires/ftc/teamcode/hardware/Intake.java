package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private static final int EXTEND_OUT = 430;
    private static final int EXTEND_LITTLE = 100;
    private static final int EXTEND_IN = 0;
    private static final double INTAKE_OPEN_POS= 0;
    private static final double INTAKE_CLOSE_POS = 1;
    private static final double INTAKE_UP_POS = 0.25;
    private static final double INTAKE_DOWN_POS = 0.77;
    private static final double INTAKE_ROTATE_90_POS = 0;
    private static final double INTAKE_ROTATE_0_POS = 0.5;

    private boolean isRotatedTo0 = true;
    private boolean isIntakeOpen;
    private boolean isIntakeUp = true;
    public Servo intakeGrab;
    public Servo intakeRotate;
    public Servo intakeFlip;
    public DcMotor extensionMotor;


    public Intake(Servo intakeGrab, Servo intakeRotate, Servo intakeFlip, DcMotor extensionMotor) {
        this.intakeGrab = intakeGrab;
        this.intakeRotate = intakeRotate;
        this.extensionMotor = extensionMotor;
        this.intakeFlip = intakeFlip;
        this.extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void openIntake() {
        intakeGrab.setPosition(INTAKE_OPEN_POS);
        isIntakeOpen = true;
    }

    public void closeIntake() {
        intakeGrab.setPosition(INTAKE_CLOSE_POS);
        isIntakeOpen = false;
    }

    public void booleanIntakeGrabber() {
        if(isIntakeOpen) {
            closeIntake();
        }
        else {
            openIntake();
        }
    }

    public void flipIntakeUp() {
        if (!isRotatedTo0) {
            rotateIntakeTo0();
        }
        if (isRotatedTo0) {
            intakeFlip.setPosition(INTAKE_UP_POS);
            isIntakeUp = true;
        }
    }

    public void flipIntakeDown() {
        intakeFlip.setPosition(INTAKE_DOWN_POS);
        isIntakeUp = false;
    }

    public void booleanFlipIntake() {
        if (isIntakeUp) {
            flipIntakeDown();
        }
        else {
            flipIntakeUp();
        }
    }

    public void rotateIntakeTo90() {
        intakeRotate.setPosition(INTAKE_ROTATE_90_POS);
        isRotatedTo0 = false;
    }

    public void rotateIntakeTo0() {
        intakeRotate.setPosition(INTAKE_ROTATE_0_POS);
        isRotatedTo0 = true;
    }

    public int getExtensionPos() {
        return extensionMotor.getCurrentPosition();
    }
    public void extendStop(){
        extensionMotor.setPower(0);
    }

    public void extend(double power){
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotor.setPower(power);
    }

    public void retract(double power){
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
