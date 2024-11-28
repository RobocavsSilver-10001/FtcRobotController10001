package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "BasicTeleOpRevisedHardwareMap")
public class BasicTeleOpRevisedHardwareMap extends LinearOpMode {

    public DcMotorEx fl, fr, bl, br;

    public DcMotorEx extendMotor;
    private double extendMotorZeroPower = 0.0;
    private double extendMotorPower = 1.0;
    private int extendMotorZeroPosition = 0;
    private int extendMotorOutPosition = -2900;

    public DcMotorEx angMotor;

    public Servo ClawClamp, ClawTurn;



    @Override
    public void runOpMode() throws InterruptedException {

        //Configuration for driver hub
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeft");
        br = hardwareMap.get(DcMotorEx.class, "BackRight");


        extendMotor = hardwareMap.get(DcMotorEx.class, "ExtendMotor");

        angMotor = hardwareMap.get(DcMotorEx.class, "AngleMotor");

        ClawClamp = hardwareMap.get(Servo.class, "ClawClamp");
        ClawTurn = hardwareMap.get(Servo.class, "ClawTurn");

        //Angle of arm stuff
        angMotor.setDirection(DcMotorEx.Direction.REVERSE);
        angMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        angMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        angMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Extend Arm Stuff
        extendMotor.setDirection(DcMotorEx.Direction.FORWARD);
        extendMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


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
                br.setPower(backLeftPower / 3);

                angMotor.setPower(backRightPower / 3);
                extendMotor.setPower(backRightPower / 3);
            } else {
                fl.setPower(frontLeftPower);
                fr.setPower(frontRightPower);
                bl.setPower(backLeftPower);
                br.setPower(backRightPower);

                angMotor.setPower(backRightPower);
                extendMotor.setPower(backRightPower);

            }

            if (gamepad1.left_trigger > .5) { //Player One angle
                angMotor.setPower(1);
                telemetry.addData("Angle Pos:", angMotor.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.right_trigger > .5) { //Player One angle
                angMotor.setPower(-1);
                telemetry.addData("Angle Pos:", angMotor.getCurrentPosition());
                telemetry.update();
            } else {
                angMotor.setPower(0);
            }

            if (gamepad2.left_trigger > .5) {
                extendMotor.setPower(-.6);
                telemetry.addData("Extend Pos:", extendMotor.getCurrentPosition());
                telemetry.update();
            } else if (gamepad2.right_trigger > .5) {
                extendMotor.setPower(.6);
                telemetry.addData("Extend Pos:", extendMotor.getCurrentPosition());
                telemetry.update();
            } else {
                extendMotor.setPower(0);
            }
            if (gamepad2.a) {
                ClawTurn.setPosition(.7);
            } else if (gamepad2.b) {
                ClawTurn.setPosition(.4);
            }
            if (gamepad2.x) {
                //.7
                ClawClamp.setPosition(.7);
            } else if (gamepad2.y) {
                //1.0
                ClawClamp.setPosition(1.0);
            }
            if (gamepad2.a) {
                extendMotor.setTargetPosition(extendMotorOutPosition);
                extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extendMotor.setPower(-1);

                telemetry.addData("Extend Pos:", extendMotor.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad2.b) {
                extendMotor.setTargetPosition(extendMotorZeroPosition);
                extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extendMotor.setPower(1);

                telemetry.addData("Extend Pos:", extendMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}




