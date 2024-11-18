package org.firstinspires.ftc.teamcode.BasicTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "BasicLinearOpMode")

public class BasicTeleOp extends LinearOpMode {

    public DcMotorEx fl, fr, bl, br;
    private DcMotorEx angMotorLeft;
    private DcMotorEx angMotorRight;
    public DcMotorEx extendMotorFront, extendMotorBack;
    public Servo ClawLeft, ClawRight, ClawMiddle;


    public void runOpMode() throws InterruptedException {

        //Configuration for driver hub
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeft");
        br = hardwareMap.get(DcMotorEx.class, "BackRight");


        extendMotorFront = hardwareMap.get(DcMotorEx.class, "ExtendMotorFront");
        extendMotorBack = hardwareMap.get(DcMotorEx.class, "ExtendMotorBack");

        angMotorLeft = hardwareMap.get(DcMotorEx.class, "AngleMotorLeft");
        angMotorRight = hardwareMap.get(DcMotorEx.class, "AngleMotorRight");

        ClawLeft = hardwareMap.get(Servo.class, "ClawLeft");
        ClawRight = hardwareMap.get(Servo.class, "ClawRight");
        ClawMiddle = hardwareMap.get(Servo.class, "ClawMiddle");


        angMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        angMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        angMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        extendMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //DT Motor directions
        fl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.FORWARD);

        // DT Motor Behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            if (gamepad1.left_bumper) { //Slow mode
                fl.setPower(frontLeftPower / 3);
                fr.setPower(frontRightPower / 3);
                bl.setPower(backLeftPower / 3);
                br.setPower(backRightPower / 3);
            } else {
                fl.setPower(frontLeftPower);
                fr.setPower(frontRightPower);
                bl.setPower(backLeftPower);
                br.setPower(backRightPower);

            }

            if (gamepad1.left_trigger > .5) { //Player One angle
                angMotorLeft.setPower(.7);
                angMotorRight.setPower(.7);
            } else if (gamepad1.right_trigger > .5) { //Player One angle
                angMotorLeft.setPower(-.7);
                angMotorRight.setPower(-.7);
            }   else {
                    angMotorRight.setPower(0);
                    angMotorLeft.setPower(0);
            }

                if (gamepad2.left_trigger > .5) {
                    extendMotorFront.setPower(-.3);
                    extendMotorBack.setPower(.3);

                } else if (gamepad2.right_trigger > .5) {
                    extendMotorFront.setPower(.3);
                    extendMotorBack.setPower(-.3);
                }
                else {
                    extendMotorBack.setPower(0);
                    extendMotorFront.setPower(0);
                }
                if (gamepad2.a) {
                    ClawLeft.setPosition(.7);
                    ClawRight.setPosition(.3);
                }
                else if (gamepad2.b) {
                    ClawLeft.setPosition(.4);
                    ClawRight.setPosition(.6);
                }
                if (gamepad2.x) {
                    //.7
                    ClawMiddle.setPosition(0.7);
                }
                else if (gamepad2.y) {
                    //1.0
                    ClawMiddle.setPosition(1);
                }

            }
        }
    }



