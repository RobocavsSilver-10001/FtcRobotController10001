package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*Tick Data
        Extend Arm Out Max = 2500 Ticks
        Extend Arm In Max = 0 Ticks (Start code with Arm all the way in)
        Claw Extended Max to floor = 0.5

 */

@TeleOp(name= "TelemetryTest")
public class ArmPresets extends LinearOpMode {
    public DcMotorEx extendMotor;

    public DcMotorEx angMotor;

    public Servo ClawGrab;
    public Servo ClawTurn;

    final double CLAW_DOWN_FLOOR_EXTEND = 0.5;
    final double CLAW_SCORE_TOP_BUCKET = 0.5;
    final double CLAW_START_POSITION = 0.35;

    final double CLAW_GRAB = 0.64;      // Fully closed
    final double CLAW_RELEASE = 0.5;  // Fully open


    final double MAX_EXTEND_PICKING_UP = 2500;
    final double MAX_EXTEND_SCORE_IN_BUCKET = 2900;
    final double EXTEND_HALF = 1500;
    final double ZERO_EXTEND = 0;

    final double ANGLE_FLOOR_PICK_UP = -1060;
    final double ANGLE_SCORE_TOP_BUKET = 3700;
    final double ANGLE_ZERO = 0;
    final double ANGLE_FULL_EXTENSION_FLOOR_PICK_UP = -850;

    /*
    _______________________________________________________________________________________
    _______________________________________________________________________________________
    */
    private void controlMotor(DcMotorEx motor, double power, boolean condition) {
        if (condition) {
            motor.setPower(power);
        } else {
            motor.setPower(0);
        }
    }
    /*
    _______________________________________________________________________________________
    _______________________________________________________________________________________
    */




    @Override
    public void runOpMode() throws InterruptedException {


        ClawGrab = hardwareMap.get(Servo.class, "ClawGrab");
        ClawTurn = hardwareMap.get(Servo.class, "ClawTurn");
        angMotor = hardwareMap.get(DcMotorEx.class, "AngleMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "ExtendMotor");


        //BRAKE
        extendMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        angMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //DIRECTION
        angMotor.setDirection(DcMotorEx.Direction.REVERSE);
        extendMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //Angle of arm stuff
        angMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        angMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Extend Arm Stuff
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Claw Stuff
        ClawTurn.setPosition(CLAW_START_POSITION);
        ClawGrab.setPosition(CLAW_GRAB);
//        ClawGrab.setPosition(0.5);
//        sleep(1000);
//        ClawGrab.setPosition(0.64);
//        sleep(1000);
//        telemetry.addData("ClawGrab Test", "Positions tested");
//        telemetry.update();


        int ANGLE_MAX = 2000; // Example max value
        int ANGLE_MIN = 0;    // Example min value
        int EXTEND_MAX = 2500;
        int EXTEND_MIN = 0;

        if (angMotor.getCurrentPosition() < ANGLE_MIN && gamepad2.left_trigger > 0.5) {
            angMotor.setPower(0);
        } else if (angMotor.getCurrentPosition() > ANGLE_MAX && gamepad2.right_trigger > 0.5) {
            angMotor.setPower(0);
        }


        waitForStart();

        if (isStopRequested()) return;
        extendMotor.setPower(0);
        angMotor.setPower(0);
        ClawTurn.setPosition(0);


        while (opModeIsActive()) {

            telemetry.addData("Gamepad2 Left Trigger:", gamepad2.left_trigger);
            telemetry.addData("Gamepad2 Right Trigger:", gamepad2.right_trigger);
            telemetry.addData("Gamepad2 A:", gamepad2.a);
            telemetry.addData("Gamepad2 B:", gamepad2.b);
            telemetry.addData("Gamepad1 Left Trigger:", gamepad1.left_trigger);
            telemetry.addData("Gamepad1 Right Trigger:", gamepad1.right_trigger);
            telemetry.addData("Angle Arm Ticks:", angMotor.getCurrentPosition());
            telemetry.addData("Arm Extension Ticks:", extendMotor.getCurrentPosition());
            telemetry.addData("Claw Turn Position:", ClawTurn.getPosition());
            telemetry.addData("Dpad Up Pressed:", gamepad2.dpad_up);
            telemetry.addData("Dpad Down Pressed:", gamepad2.dpad_down);
            telemetry.addData("ClawGrab Position:", ClawGrab.getPosition());
            telemetry.update();



            /*FIND POSITIONS
            _______________________________________________________________________________________
            _______________________________________________________________________________________
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */


            /*  ARM ANGLE
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */
            if (gamepad2.left_trigger > 0.5) {
                angMotor.setPower(1);

                telemetry.addData("Angle Pos:", angMotor.getCurrentPosition());
                telemetry.update();
            } else if (gamepad2.right_trigger > 0.5) {
                angMotor.setPower(-1);

                telemetry.addData("Angle Pos:", angMotor.getCurrentPosition());
                telemetry.update();
            } else {
                angMotor.setPower(0);
            }
            /*
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */
            /*  ARM EXTENSION
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */


            if (gamepad2.left_bumper) {
                extendMotor.setPower(1); // Extend
            } else if (gamepad2.right_bumper) {
                extendMotor.setPower(-1); // Retract
            } else {
                extendMotor.setPower(0);
            }
            /*
            int currentExtendPosition = extendMotor.getCurrentPosition();
            extendMotor.setTargetPosition(currentExtendPosition);
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendMotor.setPower(0.0001); // Low power to hold position

             */
            /*
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */
            /*  Claw Turn
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */

            if (gamepad2.a) {
                ClawTurn.setPosition(0.54);
            } else if (gamepad2.b) {
                ClawTurn.setPosition(CLAW_DOWN_FLOOR_EXTEND);
            }

            if (gamepad2.dpad_up) {
                double increase_claw_angle = 0.01;
                if (ClawTurn.getPosition() < 1) {
                    increase_claw_angle = 0.0005;
                } else {
                    increase_claw_angle = 0.0;
                }
                telemetry.addData("ClawTurn Inc amount:", increase_claw_angle);
                telemetry.addData("ClawTurn Inc Position:", ClawTurn.getPosition());
                telemetry.update();
                ClawTurn.setPosition(ClawTurn.getPosition() + increase_claw_angle);
            } else if (gamepad2.dpad_down) {
                double decrease_claw_angle = 0.01;
                if (ClawTurn.getPosition() > 0) {
                    decrease_claw_angle = 0.0005;
                } else {
                    decrease_claw_angle = 0.0;
                }
                telemetry.addData("ClawTurn Dec amount:", decrease_claw_angle);
                telemetry.addData("ClawTurn Dec Position:", ClawTurn.getPosition());
                telemetry.update();
                ClawTurn.setPosition(ClawTurn.getPosition() - decrease_claw_angle);
            }
            /*
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */
            /*  Claw Grab
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */

            if (gamepad2.x) {
                ClawGrab.setPosition(CLAW_GRAB); // Close claw
            } else if (gamepad2.y) {
                ClawGrab.setPosition(CLAW_RELEASE); // Open claw
            }

            /*
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */
            /*
            _______________________________________________________________________________________
            _______________________________________________________________________________________
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */




        }
    }
}
