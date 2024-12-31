package org.firstinspires.ftc.teamcode.Testing;

// Version 2.0.0

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/***************************************************************************************************************************
 Real-time PID Tuning: Now, the p, i, and d values are dynamically updated in the runOpMode method, using sliders from the FTC Dashboard.
 You'll be able to change the PID constants while the robot is running.
 Multiple Telemetry: Using MultipleTelemetry, the telemetry is sent both to the robot's standard output and to the FTC Dashboard for visualization.
 PID Constant Update: The PID controller’s p, i, and d values are updated in real time by the FTC Dashboard.
 ***************************************************************************************************************************/

@TeleOp(name= "IntakeTest")
public class IntakeTest extends LinearOpMode {

    // Declare hardware components
    public DcMotorEx fl, fr, bl, br; // Drive motors
    public DcMotorEx extendMotor, angMotor; // Arm motors
    public Servo ClawTurn, IntakeRight, IntakeLeft; // Claw servos

    // Constants for claw positions
    final double CLAW_DOWN_FLOOR_EXTEND = 0.5;
    final double CLAW_SCORE_TOP_BUCKET = 0.5;
    final double CLAW_HOME_POSITION = 0.5;
    final double CLAW_SPECIMEN_PICK_UP = 0.5;
    final double CLAW_CLIPPING_POSITION = 0.5094;
    final double CLAW_GRAB = 0.65;      // Fully closed
    final double CLAW_RELEASE = 0.55;  // Fully open

    // Arm extension positions
    final double MAX_EXTEND_PICKING_UP = 2500;
    final double MAX_EXTEND_SCORE_IN_BUCKET = 2900;
    final double EXTEND_HALF = 1500;
    final double ZERO_EXTEND = 0;
    final double EXTEND_POST_CLIPPING = 1000; //originally 900

    // Arm angle positions
    final double ANGLE_FLOOR_PICK_UP = -1060;
    final double ANGLE_SCORE_TOP_BUKET = 3800;
    final double MIN_ARM_ANGLE = -1070;
    final double MAX_ARM_ANGLE = 5000;
    final double ANGLE_ZERO = 0;
    final double ANGLE_SPECIMEN_FLOOR_PICK_UP = -3305;
    final double ANGLE_ARM_CLIP = 3100; //originally 2950

    // PID control variables for the extend motor
    private PIDController pidController;
    public static double p = 0.0033, i = 0, d = 0.0001; // PID constants
    public static double f = 0.001; //Gravity hold

    // Boolean flags for preset states
    boolean preset = true;
    boolean changed = false;
    boolean on = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeft");
        br = hardwareMap.get(DcMotorEx.class, "BackRight");

        CRServo IntakeRight = hardwareMap.get(CRServo.class, "IntakeRight");
        CRServo IntakeLeft = hardwareMap.get(CRServo.class, "IntakeLeft");
        ClawTurn = hardwareMap.get(Servo.class, "ClawTurn");
        angMotor = hardwareMap.get(DcMotorEx.class, "AngleMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "ExtendMotor");

        // Configure motor behavior
        extendMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        angMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        angMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setDirection(DcMotorEx.Direction.FORWARD);
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.REVERSE);


        // Reset encoders
        angMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        angMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Handle drive train control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Drive motor control with speed adjustment via bumpers
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
            telemetry.addData("Intake code: ", gamepad1.a);
            if (gamepad1.a) {
                telemetry.addData("in the right trigger", gamepad1.right_trigger);
                IntakeLeft.setPower(1);
                IntakeRight.setPower(-1);
            } else if (gamepad1.b) {
                IntakeLeft.setPower(-1);
                IntakeRight.setPower(1);
            } else {
                IntakeLeft.setPower(0);
                IntakeRight.setPower(0);
            }

            if (gamepad1.x) {
                ClawTurn.setPosition(ClawTurn.getPosition()+0.001);
            } else if (gamepad1.y) {
                ClawTurn.setPosition(ClawTurn.getPosition()-0.001);
            } else {
                ClawTurn.setPosition(ClawTurn.getPosition());
            }


            if (gamepad1.dpad_up) {
                angMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                angMotor.setPower(-1);
            } else if (gamepad1.dpad_right) {
                extendMotor.setPower(1);
            } else if (gamepad1.dpad_left) {
                extendMotor.setPower(-1);
            } else {
                extendMotor.setPower(0);
                angMotor.setPower(0);
            }
        }
    }
}