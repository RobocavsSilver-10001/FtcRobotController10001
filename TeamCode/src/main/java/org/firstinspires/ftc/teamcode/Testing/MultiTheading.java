package org.firstinspires.ftc.teamcode.Testing;

// Version 2.0.0

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/***************************************************************************************************************************
 Real-time PID Tuning: Now, the p, i, and d values are dynamically updated in the runOpMode method, using sliders from the FTC Dashboard.
 You'll be able to change the PID constants while the robot is running.
 Multiple Telemetry: Using MultipleTelemetry, the telemetry is sent both to the robot's standard output and to the FTC Dashboard for visualization.
 PID Constant Update: The PID controller’s p, i, and d values are updated in real time by the FTC Dashboard.
 ***************************************************************************************************************************/

@TeleOp(name= "MultiThreadTeleOp")
public class MultiTheading extends LinearOpMode {

    // Declare hardware components
    public DcMotorEx fl, fr, bl, br; // Drive motors
    public DcMotorEx extendMotor, angMotor; // Arm motors
    public Servo ClawGrab, ClawTurn; // Claw servos

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

        ClawGrab = hardwareMap.get(Servo.class, "ClawGrab");
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

        // Initialize PID controller for extend motor
        pidController = new PIDController(p, i, d);

        // Reset encoders
        angMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        angMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize claw positions
        //ClawTurn.setPosition(CLAW_HOME_POSITION);
        ClawGrab.setPosition(CLAW_GRAB);

        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        // Define and start threads for controlling drive and preset
        Thread driveThread = new Thread(new Runnable() {
            @Override
            public void run() {
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
                    // Call claw control function
                    controlClaw();

                    // Call viper slide control function
                    viperSlideControl();

                    // Call angle motor control function
                    controlAngleMotor();
                }
            }
        });

        Thread presetThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    // Adjust PID constants in real time using the dashboard sliders
                    pidController.setP(p);
                    pidController.setI(i);
                    pidController.setD(d);
                    pidController.setF(f);



                    // PID control for holding the arm in position
                    double currentPosition = extendMotor.getCurrentPosition();
                    double targetPosition = gamepad2.left_stick_y * MAX_EXTEND_PICKING_UP; // Adjust target based on joystick input
                    double power = pidController.calculate (currentPosition, targetPosition);
                    extendMotor.setPower(power); // Apply calculated power to hold position

                    // Display telemetry
                    telemetry.addData("PID P", p);
                    telemetry.addData("PID I", i);
                    telemetry.addData("PID D", d);
                    telemetry.addData("PID F", f);
                    telemetry.addData("Extend Motor Position", currentPosition);
                    telemetry.addData("Target Position", targetPosition);
                    telemetry.addData("PID Output", power);
                    telemetry.update();

                    // Preset control
                    if (gamepad2.a) { // Use Case: Score in top bucket use case
                        scoreInTopBucket();
                    }
                    if (gamepad2.b) { // Use Case: Zero position
                        zeroPosition();
                    }
                    if (gamepad2.x) { // Use Case: Pick up from ground
                        pickUpFromGround();
                    }
                    if (gamepad2.dpad_left) { // Use Case: Pick up specimen
                        pickUpSpecimen();
                    }
                    if (gamepad2.dpad_down) { // Use Case: Clip specimen on top bar
                        clipSpecimenWait();
                    }
                    if (gamepad2.dpad_up) { // Use Case: Clip specimen on high bar
                        clipSpecimenHighBar();
                    }
                }
            }
        });

        // Start both threads
        driveThread.start();
        presetThread.start();

        // Wait for threads to complete
        try {
            driveThread.join();
            presetThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // Function to open and close the claw
    private void controlClaw() {
        if (gamepad1.x && !changed) if (!on) {
            ClawGrab.setPosition(CLAW_GRAB); // Close claw
            sleep(500);
            on = true;
            changed = true;
        } else {
            ClawGrab.setPosition(CLAW_RELEASE); // Open claw
            sleep(500);
            on = false;
            changed = true;
        } else {
            changed = false;
        }
    }

    // Function to control the viper slide
    private void viperSlideControl() {
        if (gamepad1.right_trigger > 0.5) {
            if (extendMotor.getCurrentPosition() > ZERO_EXTEND) {
                extendArmToPosition(extendMotor.getCurrentPosition() - 5);
                waitForArmExtendPosition();
            }
        } else if (gamepad1.left_trigger > 0.5) {
            double currentAngle = angMotor.getCurrentPosition();
            double maxExtendLimit = (currentAngle < ANGLE_SCORE_TOP_BUKET - 1000) ? MAX_EXTEND_SCORE_IN_BUCKET : MAX_EXTEND_PICKING_UP;
            if (extendMotor.getCurrentPosition() < maxExtendLimit) {
                extendArmToPosition(extendMotor.getCurrentPosition() + 5);
                waitForArmExtendPosition();
            }
        }
    }

    // Function to control the angle motor
    private void controlAngleMotor() {
        if (gamepad2.left_stick_y < 0) {
            int targetPosition = angMotor.getCurrentPosition() - 5;
            if (targetPosition > MIN_ARM_ANGLE) {
                moveArmToPosition(targetPosition);
                waitForArmAngle();
            }
        } else if (gamepad2.left_stick_y > 0) {
            int targetPosition = angMotor.getCurrentPosition() + 5;
            if (targetPosition < MAX_ARM_ANGLE) {
                moveArmToPosition(targetPosition);
                waitForArmAngle();
            }
        }
    }

    // Function to score in the top bucket
    private void scoreInTopBucket() {
        ClawGrab.setPosition(CLAW_GRAB);
        sleep(100);
        ClawTurn.setPosition(CLAW_SCORE_TOP_BUCKET);
        extendArmToPosition(EXTEND_HALF);
        moveArmToPosition(ANGLE_SCORE_TOP_BUKET);
        waitForArmExtendAndAnglePosition();
        extendArmToPosition(MAX_EXTEND_SCORE_IN_BUCKET);
        waitForArmExtendPosition();
    }

    // Function to move the arm to zero position
    private void zeroPosition() {
        extendArmToPosition(ZERO_EXTEND);
        moveArmToPosition(ANGLE_ZERO);
        waitForArmExtendAndAnglePosition();
    }

    // Function to pick up from the ground
    private void pickUpFromGround() {
        ClawGrab.setPosition(CLAW_RELEASE);
        ClawTurn.setPosition(CLAW_DOWN_FLOOR_EXTEND);
        extendArmToPosition(ZERO_EXTEND); //test
        moveArmToPosition(ANGLE_FLOOR_PICK_UP);
        waitForArmExtendAndAnglePosition();
        extendArmToPosition(MAX_EXTEND_PICKING_UP);
        waitForArmExtendPosition();
    }

    // Function to pick up a specimen
    private void pickUpSpecimen() {
        ClawTurn.setPosition(CLAW_SPECIMEN_PICK_UP);
        ClawGrab.setPosition(CLAW_RELEASE);
        extendArmToPosition(ZERO_EXTEND);
        moveArmToPosition(ANGLE_SPECIMEN_FLOOR_PICK_UP);
        waitForArmExtendAndAnglePosition();
    }

    // Function to clip the specimen on the top bar
    private void clipSpecimenWait() {
        ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
        ClawGrab.setPosition(CLAW_GRAB);
        extendArmToPosition(ZERO_EXTEND);
        moveArmToPosition(ANGLE_ARM_CLIP);
        waitForArmExtendAndAnglePosition();
    }

    // Function to clip the specimen on the high bar
    private void clipSpecimenHighBar() {
        ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
        ClawGrab.setPosition(CLAW_GRAB);
        moveArmToPosition(ANGLE_ARM_CLIP);
        extendArmToPosition(EXTEND_POST_CLIPPING);
        waitForArmExtendAndAnglePosition();
        sleep(100);
        ClawGrab.setPosition(CLAW_RELEASE);
    }

    // Helper function to extend the arm to a target position
    private void extendArmToPosition(double position) {
        extendMotor.setTargetPosition((int) position);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(1);
    }

    // Helper function to move the arm to a target angle
    private void moveArmToPosition(double angle) {
        angMotor.setTargetPosition((int) angle);
        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        angMotor.setPower(1);
    }

    // Helper function to wait for the arm to extend to the target position
    private void waitForArmExtendAndAnglePosition() {
        while (extendMotor.isBusy()||angMotor.isBusy()) {
            // Wait for the arm to reach target
        }
        extendMotor.setPower(0);
        angMotor.setPower(0);
    }

    // Helper function to wait for the arm to extend to the target position
    private void waitForArmExtendPosition() {
        while (extendMotor.isBusy()) {
            // Wait for the arm to reach target
        }
        extendMotor.setPower(0);
    }

    // Helper function to wait for the arm to move to the target angle
    private void waitForArmAngle() {
        while (angMotor.isBusy()) {
            // Wait for the arm to reach target
        }
        angMotor.setPower(0);
    }
}
