package org.firstinspires.ftc.teamcode.BasicTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name= "BasicLinearOpMode")

public class BasicTeleOp extends LinearOpMode {

    public DcMotorEx fl, fr, bl, br;
    private DcMotorEx angMotorLeft;
    private DcMotorEx angMotorRight;
    public DcMotorEx extendMotorFront, extendMotorBack;


    public void runOpMode() throws InterruptedException {

        //Configuration for driver hub
        fl = hardwareMap.get(DcMotorEx.class,"FrontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = hardwareMap.get(DcMotorEx.class,"BackLeft");
        br = hardwareMap.get(DcMotorEx.class,"BackRight");

        extendMotorFront = hardwareMap.get(DcMotorEx.class, "ExtendMotorFront");
        extendMotorBack = hardwareMap.get(DcMotorEx.class, "ExtendMotorBack");

        angMotorLeft = hardwareMap.get(DcMotorEx.class, "AngleMotorLeft");
        angMotorRight = hardwareMap.get(DcMotorEx.class, "AngleMotorRight");

        angMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        angMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        extendMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        extendMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

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


            if (gamepad1.left_bumper) {
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

        }
    }
}


