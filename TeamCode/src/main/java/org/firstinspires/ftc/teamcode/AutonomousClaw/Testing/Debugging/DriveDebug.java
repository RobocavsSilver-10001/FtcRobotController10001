package org.firstinspires.ftc.teamcode.AutonomousClaw.Testing.Debugging;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PathDebugging")
public class DriveDebug extends LinearOpMode {

    public DcMotorEx fl, fr, bl, br;

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeft");
        br = hardwareMap.get(DcMotorEx.class, "BackRight");

        // Configure motor behavior
        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        fl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("Front Left Motor", fl.getCurrentPosition());
            telemetry.addData("Front Right Motor", fr.getCurrentPosition());
            telemetry.addData("Back Left Motor", bl.getCurrentPosition());
            telemetry.addData("Back Right Motor", br.getCurrentPosition());

            telemetry.update();
            // Handle drive train control
            double y = -gamepad1.left_stick_y;
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
        }
    }
}

