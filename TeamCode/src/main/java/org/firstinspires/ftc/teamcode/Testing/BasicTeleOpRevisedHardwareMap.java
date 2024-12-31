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

@TeleOp(name= "skibidi")
public class BasicTeleOpRevisedHardwareMap extends LinearOpMode {


    public DcMotorEx fl, fr, bl, br;

    public DcMotorEx extendMotor;

    public DcMotorEx angMotor;

    public Servo ClawGrab;
    public Servo ClawTurn;

    final double CLAW_DOWN_FLOOR_EXTEND = 0.5;
    final double CLAW_SCORE_TOP_BUCKET = 0.5;
    final double CLAW_HOME_POSITION = 0.5;
    final double CLAW_SPECIMEN_PICK_UP = 0.5;
    final double CLAW_CLIPPING_POSITION = 0.52;

    final double CLAW_GRAB = 0.65;      // Fully closed
    final double CLAW_RELEASE = 0.55;  // Fully open


    final double MAX_EXTEND_PICKING_UP = 2500;
    final double MAX_EXTEND_SCORE_IN_BUCKET = 2900;
    final double EXTEND_HALF = 1500;
    final double ZERO_EXTEND = 0;
    final double EXTEND_POST_CLIPPING = 900; //originaly 715

    final double ANGLE_FLOOR_PICK_UP = -1060;
    //final double ANGLE_SCORE_TOP_BUKET = 3700;
    final double ANGLE_SCORE_TOP_BUKET = 3800;
    final double ANGLE_ZERO = 0;
    final double ANGLE_FULL_EXTENSION_FLOOR_PICK_UP = -850;
    final double ANGLE_SPECIMEN_FLOOR_PICK_UP = -3305;
    final double ANGLE_ARM_CLIP = 2950;
    boolean preset = true;
    boolean changed = false;
    boolean on = false;

    private PIDController controller;
    public static double p = 0.0033, i = 0, d = 0.0001;
    public static double f = 0.055;




    @Override
    public void runOpMode() throws InterruptedException {

        //Configuration for driver hub
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeft");
        br = hardwareMap.get(DcMotorEx.class, "BackRight");

        telemetry.addData("Status", "Initialized");
        telemetry.update();


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
        ClawTurn.setPosition(CLAW_HOME_POSITION);
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

       /*
        if (angMotor.getCurrentPosition() < ANGLE_MIN && gamepad2.left_trigger > 0.5) {
            angMotor.setPower(0);
        } else if (angMotor.getCurrentPosition() > ANGLE_MAX && gamepad2.right_trigger > 0.5) {
            angMotor.setPower(0);
        }


        */

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

            telemetry.clearAll();


            //Manual Extending
            if (gamepad1.left_trigger > 0.5) {
                extendMotor.setPower(1); // Extend
            } else if (gamepad1.right_trigger > 0.5) {
                extendMotor.setPower(-1); // Retract
            } else {
                extendMotor.setPower(0);
            }

            //Manual Angle
            if (gamepad2.left_trigger > 0.5) {
                angMotor.setPower(1);

            } else if (gamepad2.right_trigger > 0.5) {
                angMotor.setPower(-1);

            } else {
                angMotor.setPower(0);
            }





            /*#######################################################################################
            #########################################################################################
            #########################################################################################
            ##############################         Presets       ####################################
            #########################################################################################
            #########################################################################################
            #######################################################################################*/


            if (gamepad2.a) {//Score In Top Bucket

                ClawTurn.setPosition(CLAW_SCORE_TOP_BUCKET);
                //Pull in arm half way

                while (extendMotor.getCurrentPosition() > EXTEND_HALF) {
                    extendMotor.setTargetPosition((int) EXTEND_HALF);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(1);
                }
                extendMotor.setPower(0);

                //Extend arm up

                while (angMotor.getCurrentPosition() < ANGLE_SCORE_TOP_BUKET) {
                    angMotor.setTargetPosition((int) ANGLE_SCORE_TOP_BUKET);
                    angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    angMotor.setPower(1);
                }
                angMotor.setPower(0);

                //Extend arm out

                while (extendMotor.getCurrentPosition() < MAX_EXTEND_SCORE_IN_BUCKET) {
                    extendMotor.setTargetPosition((int) MAX_EXTEND_SCORE_IN_BUCKET);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(1);
                }
                extendMotor.setPower(0.6);
            }

            if (gamepad2.b) {//Zero Position

                while (extendMotor.getCurrentPosition() > ZERO_EXTEND) {
                    extendMotor.setTargetPosition(0);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(-1);
                }
                extendMotor.setPower(-0.0006);

                //Extend arm up

                while (angMotor.getCurrentPosition() > ANGLE_ZERO || angMotor.getCurrentPosition() < ANGLE_ZERO) {
                    angMotor.setTargetPosition(0);
                    angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    angMotor.setPower(1);
                }
                angMotor.setPower(0);
            }


            if (gamepad2.x) {//Pick up from ground

                //Arm goes down

                while (angMotor.getCurrentPosition() != ANGLE_FLOOR_PICK_UP) {
                    angMotor.setTargetPosition((int) ANGLE_FLOOR_PICK_UP);
                    angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    angMotor.setPower(1);
                }
                angMotor.setPower(0);

                //Extend out

                while (extendMotor.getCurrentPosition() < MAX_EXTEND_PICKING_UP) {
                    extendMotor.setTargetPosition((int) MAX_EXTEND_PICKING_UP);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(1);
                }
                extendMotor.setPower(0);

                ClawGrab.setPosition(CLAW_RELEASE);
                ClawTurn.setPosition(CLAW_DOWN_FLOOR_EXTEND);
            }

            if (gamepad2.dpad_left) {//This will be picking up specimen
                preset = true;

                ClawTurn.setPosition(CLAW_SPECIMEN_PICK_UP);
                ClawGrab.setPosition(CLAW_RELEASE);

                while (extendMotor.getCurrentPosition() > ZERO_EXTEND) {
                    extendMotor.setTargetPosition(0);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(1);
                }

                while (angMotor.getCurrentPosition() > ANGLE_SPECIMEN_FLOOR_PICK_UP) {
                    angMotor.setTargetPosition((int) ANGLE_SPECIMEN_FLOOR_PICK_UP);
                    angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    angMotor.setPower(1);
                }
            }

            if (gamepad2.dpad_down) {//This will be WAITING to clip the specimen on TOP BAR
                preset = true;

                ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
                ClawGrab.setPosition(CLAW_GRAB);

                while (extendMotor.getCurrentPosition() > ZERO_EXTEND) {
                    extendMotor.setTargetPosition(0);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(1);
                }

                while (angMotor.getCurrentPosition() != ANGLE_ARM_CLIP) {
                    angMotor.setTargetPosition((int) ANGLE_ARM_CLIP);
                    angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    angMotor.setPower(1);
                }
            }

            if (gamepad2.dpad_up) {//This will be clipping specimen on HIGH BAR
                preset = true;

                ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
                ClawGrab.setPosition(CLAW_GRAB);

                while (angMotor.getCurrentPosition() != ANGLE_ARM_CLIP) {
                    angMotor.setTargetPosition((int) ANGLE_ARM_CLIP);
                    angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    angMotor.setPower(1);
                }
                while (extendMotor.getCurrentPosition() != EXTEND_POST_CLIPPING) {
                    extendMotor.setTargetPosition((int) EXTEND_POST_CLIPPING);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(1);
                }
                sleep(500);
                ClawGrab.setPosition(CLAW_RELEASE);
            }



            /*#######################################################################################
            #########################################################################################
            #########################################################################################
            ################################   END OF PRESENTS   ####################################
            #########################################################################################
            #########################################################################################
            #######################################################################################*/

                /*  Claw Turn
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */

            if (gamepad1.dpad_up) {
                ClawTurn.setPosition(0.54);
            } else if (gamepad1.dpad_right) {
                ClawTurn.setPosition(CLAW_DOWN_FLOOR_EXTEND);
            }

            //Manual Claw Turn

            if (gamepad1.dpad_up) {
                double increase_claw_angle = 0.01;
                if (ClawTurn.getPosition() < 0.6) {
                    increase_claw_angle = 0.0005;
                } else {
                    increase_claw_angle = 0.0;
                }
                telemetry.addData("ClawTurn Inc amount:", increase_claw_angle);
                telemetry.addData("ClawTurn Inc Position:", ClawTurn.getPosition());
                telemetry.update();
                ClawTurn.setPosition(ClawTurn.getPosition() + increase_claw_angle);
            } else if (gamepad1.dpad_down) {
                double decrease_claw_angle = 0.01;
                if (ClawTurn.getPosition() > 0.35) {
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
    }
}




