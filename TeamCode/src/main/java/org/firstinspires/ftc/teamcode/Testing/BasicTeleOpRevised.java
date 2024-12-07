package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;

/***************************************************************************************************************************
 Real-time PID Tuning: Now, the p, i, and d values are dynamically updated in the runOpMode method, using sliders from the FTC Dashboard.
 You'll be able to change the PID constants while the robot is running.
 Multiple Telemetry: Using MultipleTelemetry, the telemetry is sent both to the robot's standard output and to the FTC Dashboard for visualization.
 PID Constant Update: The PID controller’s p, i, and d values are updated in real time by the FTC Dashboard.
 ***************************************************************************************************************************/

@TeleOp(name= "BasicTeleOpRevised")
public class BasicTeleOpRevised extends LinearOpMode {

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
    final double CLAW_GRAB = 0.69;      // Fully closed
    final double CLAW_RELEASE = 0.55;  // Fully open

    // Arm extension positions
    final double MAX_EXTEND_PICKING_UP = 2500;
    final double MAX_EXTEND_SCORE_IN_BUCKET = 2900;
    final double EXTEND_HALF = 1500;
    final double ZERO_EXTEND = 0;
    final double EXTEND_POST_CLIPPING = 1000; //originally 900

    // Arm angle positions
    final double ANGLE_FLOOR_PICK_UP = -950;
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
        ClawGrab.setPosition(CLAW_GRAB);

        // Initialize telemetry for Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for start
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Adjust PID constants in real time using the dashboard sliders
            pidController.setP(p);
            pidController.setI(i);
            pidController.setD(d);
            pidController.setF(f);

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

            // Call arm control function
            controlArm();

            // Call angle motor control function
            controlAngleMotor();

            // PID control for holding the arm in position
            double currentPosition = extendMotor.getCurrentPosition();
            double targetPosition = gamepad2.left_stick_y * MAX_EXTEND_PICKING_UP; // Adjust target based on joystick input
            double power = pidController.calculate(currentPosition, targetPosition);
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
            if (gamepad2.dpad_down) { // Use Case: Waiting to clip specimen on high bar
                clipSpecimenWait();
            }
            if (gamepad2.dpad_up) { // Use Case: Clip specimen on high bar
                clipSpecimenHighBar();
            }
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
    // Function to control the arm
    private void controlArm() {
        if (gamepad1.right_trigger > 0.5) {
            // Prevent extending past the zero position (do not retract past 0)
            if (extendMotor.getCurrentPosition() > ZERO_EXTEND) {
                extendMotor.setPower(-1); // Retract the extend motor
            } else {
                extendMotor.setPower(0);  // Stop the motor if at or past zero
            }
        } else if (gamepad1.left_trigger > 0.5) {
            // Get the current arm angle position
            double currentAngle = angMotor.getCurrentPosition();

            // Determine the max extension limit based on arm angle
            double maxExtendLimit;
            if (currentAngle < ANGLE_SCORE_TOP_BUKET - 1000) {
                maxExtendLimit = MAX_EXTEND_SCORE_IN_BUCKET; // Use max limit for high angle
            } else {
                maxExtendLimit = MAX_EXTEND_PICKING_UP; // Use max limit for low angle
            }

            if (extendMotor.getCurrentPosition() < maxExtendLimit) {
                extendMotor.setPower(1); // Extend the arm
            } else {
                extendMotor.setPower(0); // Stop the arm when the limit is reached
            }
        } else {
            extendMotor.setPower(0); // Stop when neither trigger is pressed
        }
    }

    // Function to control the angle motor
    private void controlAngleMotor() {
        if (gamepad2.left_stick_y < 0) {
            // Move arm up if joystick pushed upwards, but limit to max arm angle
            int targetPosition = angMotor.getCurrentPosition() - 5;
            if (targetPosition > MIN_ARM_ANGLE) {
                angMotor.setTargetPosition(targetPosition);
                angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                angMotor.setPower(0.5);
            } else {
                angMotor.setPower(0);  // Stop motor when the limit is reached
            }
        } else if (gamepad2.left_stick_y > 0) {
            // Move arm down if joystick pushed downwards, but limit to min arm angle
            int targetPosition = angMotor.getCurrentPosition() + 5;
            if (targetPosition < MAX_ARM_ANGLE) {
                angMotor.setTargetPosition(targetPosition);
                angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                angMotor.setPower(0.5);
            } else {
                angMotor.setPower(0);  // Stop motor when the limit is reached
            }
        } else {
            angMotor.setPower(0);  // Stop motor when no movement is requested
        }
    }

    // Additional methods for preset actions (e.g., score in top bucket, zero position, etc.)
/*
    // Function to score in the top bucket
    private void scoreInTopBucket() {
        // Move the arm to the top scoring position
        angMotor.setTargetPosition((int) ANGLE_SCORE_TOP_BUKET);
        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        angMotor.setPower(0.5);

        // Extend the arm to reach the top bucket
        extendMotor.setTargetPosition((int) MAX_EXTEND_SCORE_IN_BUCKET);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(0.5);

        // Activate the claw to score the object
        ClawTurn.setPosition(CLAW_SCORE_TOP_BUCKET);
        ClawGrab.setPosition(CLAW_GRAB);
    }

    // Function to move the arm to zero position
    private void zeroPosition() {
        // Move the arm to the zero position
        angMotor.setTargetPosition((int) ANGLE_ZERO);
        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        angMotor.setPower(0.5);

        // Retract the arm fully
        extendMotor.setTargetPosition((int) ZERO_EXTEND);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(0.5);

        // Open the claw
        ClawTurn.setPosition(CLAW_HOME_POSITION);
    }

    // Function to pick up from the ground
    private void pickUpFromGround() {
        // Move the arm to the position for picking up from the ground
        angMotor.setTargetPosition((int) ANGLE_FLOOR_PICK_UP);
        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        angMotor.setPower(0.5);

        // Extend the arm to reach the ground
        extendMotor.setTargetPosition((int) MAX_EXTEND_PICKING_UP);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(0.5);

        // Open the claw to grab the object
        ClawTurn.setPosition(CLAW_DOWN_FLOOR_EXTEND);
        ClawGrab.setPosition(CLAW_RELEASE);
    }

    // Function to pick up a specimen
    private void pickUpSpecimen() {
        // Move the arm to the position for picking up the specimen
        angMotor.setTargetPosition((int) ANGLE_SPECIMEN_FLOOR_PICK_UP);
        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        angMotor.setPower(0.5);

        // Extend the arm to pick up the specimen
        extendMotor.setTargetPosition((int) MAX_EXTEND_PICKING_UP);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(0.5);

        // Activate the claw to grab the specimen
        ClawTurn.setPosition(CLAW_SPECIMEN_PICK_UP);
        ClawGrab.setPosition(CLAW_GRAB);
    }

    // Function to clip the specimen on the top bar
    private void clipSpecimenWait() {
        // Move the arm to the clipping position
        angMotor.setTargetPosition((int) ANGLE_ARM_CLIP);
        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        angMotor.setPower(0.5);

        // Extend the arm for clipping the specimen onto the top bar
        extendMotor.setTargetPosition((int) EXTEND_POST_CLIPPING);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(0.5);

        // Activate the claw to clip the specimen
        ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
        ClawGrab.setPosition(CLAW_GRAB);
    }

    // Function to clip the specimen on the high bar
    private void clipSpecimenHighBar() {
        // Move the arm to the clipping position
        angMotor.setTargetPosition((int) ANGLE_ARM_CLIP);
        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        angMotor.setPower(0.5);

        // Extend the arm for clipping the specimen onto the high bar
        extendMotor.setTargetPosition((int) EXTEND_HALF);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(0.5);

        // Activate the claw to clip the specimen
        ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
        ClawGrab.setPosition(CLAW_GRAB);

        // Release the specimen after clipping
        sleep(500);  // wait for some time to ensure the specimen is clipped
        ClawGrab.setPosition(CLAW_RELEASE);
    }
 */
// #############################################################################
// Take a look at the below functions if the above doesn't work out.
// #############################################################################
    // Function to score in the top bucket
    private void scoreInTopBucket() {
        ClawTurn.setPosition(CLAW_SCORE_TOP_BUCKET);
        extendArmToPosition(EXTEND_HALF);
        moveArmToPosition(ANGLE_SCORE_TOP_BUKET);
        extendArmToPosition(MAX_EXTEND_SCORE_IN_BUCKET);
    }

    // Function to move the arm to zero position
    private void zeroPosition() {
        extendArmToPosition(ZERO_EXTEND);
        moveArmToPosition(ANGLE_ZERO);
    }

    // Function to pick up from the ground
    private void pickUpFromGround() {
        ClawGrab.setPosition(CLAW_RELEASE);
        ClawTurn.setPosition(CLAW_DOWN_FLOOR_EXTEND);
        extendArmToPosition(ZERO_EXTEND); //test
        moveArmToPosition(ANGLE_FLOOR_PICK_UP);
        extendArmToPosition(MAX_EXTEND_PICKING_UP);
    }

    // Function to pick up a specimen
    private void pickUpSpecimen() {
        ClawTurn.setPosition(CLAW_SPECIMEN_PICK_UP);
        ClawGrab.setPosition(CLAW_RELEASE);
        extendArmToPosition(ZERO_EXTEND);
        moveArmToPosition(ANGLE_SPECIMEN_FLOOR_PICK_UP);
    }

    // Function to clip the specimen on the top bar
    private void clipSpecimenWait() {
        ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
        ClawGrab.setPosition(CLAW_GRAB);
        extendArmToPosition(ZERO_EXTEND);
        moveArmToPosition(ANGLE_ARM_CLIP);
    }

    // Function to clip the specimen on the high bar
    private void clipSpecimenHighBar() {
        ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
        ClawGrab.setPosition(CLAW_GRAB);
        moveArmToPosition(ANGLE_ARM_CLIP);
        extendArmToPosition(EXTEND_POST_CLIPPING);
        sleep(200);
        ClawGrab.setPosition(CLAW_RELEASE);
    }

    // Helper function to extend the arm to a target position
    private void extendArmToPosition(double position) {
        extendMotor.setTargetPosition((int) position);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extendMotor.setPower(1);
        while (extendMotor.isBusy()) {
            // Wait for the arm to reach target
        }
        extendMotor.setPower(0);
    }

    // Helper function to move the arm to a target angle
    private void moveArmToPosition(double angle) {
        angMotor.setTargetPosition((int) angle);
        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        angMotor.setPower(1);
        while (angMotor.isBusy()) {
            // Wait for the arm to reach target
        }
        angMotor.setPower(0);
    }
}
