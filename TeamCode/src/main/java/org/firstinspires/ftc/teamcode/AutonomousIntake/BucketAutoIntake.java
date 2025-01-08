package org.firstinspires.ftc.teamcode.AutonomousIntake;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DefaultRRFiles.MecanumDrive;

@Autonomous(name = "BucketAutoIntake3")
public class BucketAutoIntake extends LinearOpMode {


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8, -60, Math.toRadians(90)));
        CRServo IntakeRight = hardwareMap.get(CRServo.class, "IntakeRight");
        CRServo IntakeLeft = hardwareMap.get(CRServo.class, "IntakeLeft");
        Servo ClawTurn = hardwareMap.servo.get("ClawTurn");
        DcMotor angMotor = hardwareMap.dcMotor.get("AngleMotor");
        DcMotor extendMotor = hardwareMap.dcMotor.get("ExtendMotor");

        angMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


    }
}
