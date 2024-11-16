package org.firstinspires.ftc.teamcode.AdvancedTeleOp.Robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    public HardwareMap hardwareMap;

    public DcMotorEx frontLeft, frontRight, backLeft, backRight; //DT Motors
    public DcMotorEx leftAngle, rightAngle; //Arm Angle Motors
    public DcMotorEx frontExtend, backExtend; //Arm Extension Motors

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
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        this.backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        //Arm Angle
        this.leftAngle = hardwareMap.get(DcMotorEx.class, "AngleMotorLeft");
        leftAngle.setDirection(DcMotorEx.Direction.REVERSE);

        this.rightAngle = hardwareMap.get(DcMotorEx.class, "AngleMotorRight");
        rightAngle.setDirection(DcMotorSimple.Direction.REVERSE);

        //Arm Extension
        this.frontExtend = hardwareMap.get(DcMotorEx.class, "ExtendMotorFront");
        frontExtend.setDirection(DcMotorEx.Direction.FORWARD);

        this.backExtend = hardwareMap.get(DcMotorEx.class, "ExtendMotorBack");
        backExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        //Odometry
        //this.leftPod =
        //this.rightPod =
        //this.middlePod =

    }
}
