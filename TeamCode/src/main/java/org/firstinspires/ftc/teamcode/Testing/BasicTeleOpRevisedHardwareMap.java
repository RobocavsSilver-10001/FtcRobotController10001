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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "BasicTeleOpRevisedHardwareMap")
public class BasicTeleOpRevisedHardwareMap extends LinearOpMode {


    public DcMotorEx fl, fr, bl, br;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx extendMotor;
    private double extendMotorZeroPower = 0.0;
    private int extendMotorZeroPosition = 0;
    private int extendMotorOutPosition = 2500;

    public DcMotorEx angMotor;
    private double angMotorZeroPower = 0.0;
    private int angMotorZeroPosition = 0;
    private int angMotorUpPosition = 5040;
    private int angMotorDownOutPosition = -930;
    private int angMotorDownPosition = -3000;

    public Servo ClawGrab, ClawTurn;





    @Override
    public void runOpMode() throws InterruptedException {

        //Configuration for driver hub
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeft");
        br = hardwareMap.get(DcMotorEx.class, "BackRight");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        extendMotor = hardwareMap.get(DcMotorEx.class, "ExtendMotor");

        angMotor = hardwareMap.get(DcMotorEx.class, "AngleMotor");

        ClawGrab = hardwareMap.get(Servo.class, "ClawGrab");
        ClawTurn = hardwareMap.get(Servo.class, "ClawTurn");

        //Angle of arm stuff
        angMotor.setDirection(DcMotorEx.Direction.REVERSE);
        angMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        angMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        angMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //Extend Arm Stuff
        extendMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        //DT Motor directions
        fl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.REVERSE);

        // DT Motor Behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double frontLeftPower  = axial - lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial + lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            fl.setPower(frontLeftPower);
            fr.setPower(frontRightPower);
            bl.setPower(backLeftPower);
            br.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();


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
                extendMotor.setPower(.6);
                telemetry.addData("Extend Tri Pos:", extendMotor.getCurrentPosition());
                telemetry.update();
            } else if (gamepad2.right_trigger > .5) {
                extendMotor.setPower(-.6);
                telemetry.addData("Extend Tri Pos:", extendMotor.getCurrentPosition());
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
                ClawGrab.setPosition(.7);
            } else if (gamepad2.y) {
                //1.0
                ClawGrab.setPosition(1.0);
            }
            if (gamepad2.a) { //Extend Arm out player 2
                extendMotor.setTargetPosition(extendMotorOutPosition);
                extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                while (extendMotor.getCurrentPosition() < extendMotor.getTargetPosition()) {
                    extendMotor.setPower(1);

                    telemetry.addData("Extend Pos:", extendMotor.getCurrentPosition());
                    telemetry.update();
                }
                extendMotor.setPower(extendMotorZeroPower);
            }

            if (gamepad2.b) { //Retract arm in player 2
                extendMotor.setTargetPosition(extendMotorZeroPosition);
                extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                while (extendMotor.getCurrentPosition() > extendMotor.getTargetPosition()) {
                    extendMotor.setPower(-1);

                    telemetry.addData("Extend Pos:", extendMotor.getCurrentPosition());
                    telemetry.update();
                }
                extendMotor.setPower(extendMotorZeroPower);
            }
            if (gamepad1.a) {
                angMotor.setTargetPosition(angMotorUpPosition);
                angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                while (angMotor.getCurrentPosition() <  angMotor.getTargetPosition()) {
                    angMotor.setPower(1);
                }
                angMotor.setPower(0);
            }

            if (gamepad1.b) {
                angMotor.setTargetPosition(angMotorDownPosition);
                angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                while (angMotor.getCurrentPosition() > angMotor.getTargetPosition()) {
                    angMotor.setPower(-1);
                }
                angMotor.setPower(0);
            }
            if (gamepad1.x) {
                angMotor.setTargetPosition(angMotorZeroPosition);
                angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                if (angMotor.getCurrentPosition() > angMotor.getTargetPosition()) {
                    angMotor.setPower(-1);
                } else if (angMotor.getCurrentPosition() < angMotor.getTargetPosition()) {
                    angMotor.setPower(1);
                }
            }
        }
    }
}




