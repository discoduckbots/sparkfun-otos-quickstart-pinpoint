package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private static final int EXTEND_OUT = 500;
    private static final int EXTEND_LITTLE = 100;
    private static final int EXTEND_IN = 0;
    private static final double INTAKE_OPEN_POS= 0.8;
    private static final double INTAKE_CLOSE_POS = 0.4;
    private static final double INTAKE_UP_POS = 0.25;
    private static final double INTAKE_DOWN_POS = 0.77;
    private static final double INTAKE_DOWN_LOWER = 0.8;
    private static final double INTAKE_ROTATE_90_POS = 1.0; //b
    private static final double INTAKE_ROTATE_0_POS = 0.0; //a

    private boolean isRotatedTo0 = true;
    private boolean isIntakeOpen;
    private boolean isIntakeUp = true;
    private boolean buttonPressIntake = false;
    private boolean buttonPressFlip = false;
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

    public void onPressIntake() {

        if (buttonPressIntake) return;
        buttonPressIntake = true;
        if (isIntakeOpen) {
            isIntakeOpen = false;
            closeIntake();
        }
        else {
            isIntakeOpen = true;
            openIntake();
        }
    }

    public void onReleaseIntake() {
        buttonPressIntake = false;
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

    public void onPressFlip() {

        if (buttonPressFlip) return;
        buttonPressFlip = true;
        if (isIntakeUp) {
            isIntakeUp = false;
            flipIntakeDown();
        }
        else {
            isIntakeUp = true;
            flipIntakeUp();
        }
    }

    public void onReleaseFlip() {
        buttonPressFlip = false;
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
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }

    public void extendLittleByEncoder(double power){
        extensionMotor.setTargetPosition(EXTEND_LITTLE);
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }

    public void retractByEncoder(double power){
        extensionMotor.setTargetPosition(EXTEND_IN);
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }

    public void holdPosition(int position, double power){
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }
}
