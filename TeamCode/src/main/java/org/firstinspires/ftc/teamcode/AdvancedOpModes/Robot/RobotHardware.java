package org.firstinspires.ftc.teamcode.AdvancedOpModes.Robot;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public HardwareMap hardwareMap;

    public DcMotorEx frontLeft, frontRight, backLeft, backRight; //DT Motors
    public DcMotorEx angMotor; //Arm Angle Motors
    public DcMotorEx extendMotor; //Arm Extension Motors
    public Servo ClawAngle; //End effector angle change
    public Servo Grabber; //Grabber
    public Encoder par0, par1, perp;

    private static RobotHardware instance = null;
    private boolean enabled;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }


    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        //DT
        this.frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);

        this.frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);

        this.backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);

        this.backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        //Angle Motor
        this.angMotor = hardwareMap.get(DcMotorEx.class, "AngleMotor");
        angMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Arm Extension
        this.extendMotor = hardwareMap.get(DcMotorEx.class, "ExtendMotor");
        extendMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //Claw stuff
        this.ClawAngle = hardwareMap.get(Servo.class, "ClawTurn");

        this.Grabber = hardwareMap.get(Servo.class, "ClawGrab");

    }
}
