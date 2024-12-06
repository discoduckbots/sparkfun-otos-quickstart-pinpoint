package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class HardwareStore {
    public DcMotorEx frontLeft ;
    public DcMotorEx frontRight ;
    public DcMotorEx backRight ;
    public DcMotorEx backLeft ;
    private Arm arm;
    public DcMotor liftLeft;
    public DcMotor liftRight;
    public DcMotor extensionMotor;
    public Intake intake;
    public CRServo intakeLeft;
    public CRServo intakeRight;
    public Grabber grabber;
    public Servo grabberServo;
    public Servo intakeFlip;
    public Servo grabberFlip;

    public PinpointDrive drive;

//    public SparkFunOTOSDrive autoDrive;

    public HardwareStore(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = new Arm(liftLeft, liftRight);

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        extensionMotor = hardwareMap.get(DcMotor.class, "extension");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new Intake(intakeLeft, intakeRight, extensionMotor);

        grabberServo = hardwareMap.get(Servo.class, "grabberServo");
        grabberFlip = hardwareMap.get(Servo.class, "grabberFlip");
        intakeFlip = hardwareMap.get(Servo.class, "intakeFlip");

        grabber = new Grabber(grabberServo, grabberFlip, intakeFlip);

        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
//        autoDrive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    public Arm getArm() {
        return arm;
    }

    public PinpointDrive getDrive(){
        return drive;
    }

    public Intake getIntake(){
        return intake;
    }

    public Grabber getGrabber(){
        return grabber;
    }

//    public SparkFunOTOSDrive getAutoDrive(){
//        return autoDrive;
//    }
}