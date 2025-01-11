package org.firstinspires.ftc.teamcode.AutonomousClaw;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GlobalHardware {
    public DcMotorEx angMotor, extendMotor;
    public Servo ClawGrab, ClawTurn;

    public GlobalHardware(HardwareMap hardwareMap) {

        Servo ClawGrab = hardwareMap.servo.get("ClawGrab");
        Servo ClawTurn = hardwareMap.servo.get("ClawTurn");
        DcMotor angMotor = hardwareMap.dcMotor.get("AngleMotor");
        DcMotor extendMotor = hardwareMap.dcMotor.get("ExtendMotor");

        angMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
