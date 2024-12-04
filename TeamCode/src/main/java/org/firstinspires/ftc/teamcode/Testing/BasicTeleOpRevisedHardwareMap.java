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

    public DcMotorEx angMotor;

    public Servo ClawGrab;
    public Servo ClawTurn;

    final double CLAW_DOWN_FLOOR_EXTEND = 0.5;
    final double CLAW_SCORE_TOP_BUCKET = 0.5;
    final double CLAW_HOME_POSITION = 0.35;
    final double CLAW_SPECIMEN_PICK_UP = 0.6;
    final double CLAW_CLIPPING_POSITION = 0.6;

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
    final double ANGLE_SPECIMEN_FLOOR_PICK_UP = -4670;
    final double ANGLE_ARM_READY_TO_CLIP = -1930;
    final double ANGLE_ARM_DONE_CLIPPING = -3950;

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

        boolean done;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Gamepad2 Left Trigger:", gamepad2.left_trigger);
            telemetry.addData("Gamepad2 Right Trigger:", gamepad2.right_trigger);
            telemetry.addData("Gamepad2 A:", gamepad2.a);
            telemetry.addData("Gamepad2 B:", gamepad2.b);
            telemetry.addData("Gamepad1 Left Trigger:", gamepad1.left_trigger);
            telemetry.addData("Gamepad1 Right Trigger:", gamepad1.right_trigger);
            telemetry.addData("Gamepad1 A:", gamepad1.a);
            telemetry.addData("Gamepad1 B:", gamepad1.b);
            telemetry.addData("Angle Arm Ticks:", angMotor.getCurrentPosition());
            telemetry.addData("Arm Extension Ticks:", extendMotor.getCurrentPosition());
            telemetry.addData("Claw Turn Position:", ClawTurn.getPosition());
            telemetry.addData("Dpad Up Pressed:", gamepad2.dpad_up);
            telemetry.addData("Dpad Down Pressed:", gamepad2.dpad_down);
            telemetry.addData("ClawGrab Position:", ClawGrab.getPosition());
            telemetry.update();


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

            telemetry.clearAll();

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

            //Manual Extending
            if (gamepad1.left_trigger > 0.5) {
                extendMotor.setPower(1); // Extend
            } else if (gamepad1.right_trigger > 0.5) {
                extendMotor.setPower(-1); // Retract
            } else {
                extendMotor.setPower(0.0006); // Hold position
            }

            //Manual Angle
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

            /*#######################################################################################
            #########################################################################################
            #########################################################################################
            ##############################         Presets       ####################################
            #########################################################################################
            #########################################################################################
            #######################################################################################*/


            if (gamepad2.a) {//Score In Top Bucket
                done = false;
                while (!done) {

                    //Pull in arm half way

                    while (extendMotor.getCurrentPosition() > EXTEND_HALF) {
                        extendMotor.setTargetPosition((int) EXTEND_HALF);
                        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        extendMotor.setPower(-1);
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
                    extendMotor.setPower(0.0006);


                    /* ########################################################
                    Closing out loop by setting "done" to true
                    ########################################################### */
                    done = true;
                }
            }




            if (gamepad2.b) {//Zero Position
                done = false;
                while (!done) {

                    //Pull in arm half way

                    while (extendMotor.getCurrentPosition() > ZERO_EXTEND) {
                        extendMotor.setTargetPosition(0);
                        extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        extendMotor.setPower(-1);
                    }
                    extendMotor.setPower(-0.0006);

                    //Extend arm up

                    while (angMotor.getCurrentPosition() > ANGLE_ZERO) {
                        angMotor.setTargetPosition(0);
                        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        angMotor.setPower(-1);
                    }
                    angMotor.setPower(0);

                    while (angMotor.getCurrentPosition() < ANGLE_ZERO) {
                        angMotor.setTargetPosition(0);
                        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        angMotor.setPower(1);
                    }
                    angMotor.setPower(0);
                    ClawTurn.setPosition(CLAW_HOME_POSITION);
                    sleep(100);
                    ClawGrab.setPosition(CLAW_GRAB);

                /* ########################################################
                Closing out loop by setting "done" to true
                ########################################################### */
                    done = true;
                }
            }


            if (gamepad2.x) {//Pick up from ground
                done = false;
                while (!done) {

                    ClawTurn.setPosition(CLAW_CLIPPING_POSITION);

                    //Arm goes down

                    while (angMotor.getCurrentPosition() > ANGLE_FLOOR_PICK_UP) {
                        angMotor.setTargetPosition((int) ANGLE_FLOOR_PICK_UP);
                        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        angMotor.setPower(-1);
                    }
                    angMotor.setPower(0);

                    while (angMotor.getCurrentPosition() < ANGLE_FLOOR_PICK_UP) {
                        angMotor.setTargetPosition((int) ANGLE_FLOOR_PICK_UP);
                        angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        angMotor.setPower(1);
                    }
                    angMotor.setPower(0);

                    //Extend out
                    if (angMotor.getCurrentPosition() != ANGLE_FLOOR_PICK_UP) {
                        while (angMotor.getCurrentPosition() > ANGLE_FLOOR_PICK_UP) {
                            angMotor.setTargetPosition((int) ANGLE_FLOOR_PICK_UP);
                            angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                            angMotor.setPower(-1);
                        }
                        angMotor.setPower(0);

                        while (angMotor.getCurrentPosition() < ANGLE_FLOOR_PICK_UP) {
                            angMotor.setTargetPosition((int) ANGLE_FLOOR_PICK_UP);
                            angMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                            angMotor.setPower(1);
                        }
                        angMotor.setPower(0);
                    } else {
                        while (extendMotor.getCurrentPosition() < MAX_EXTEND_PICKING_UP) {
                            extendMotor.setTargetPosition((int) MAX_EXTEND_PICKING_UP);
                            extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                            extendMotor.setPower(1);
                        }
                        extendMotor.setPower(0);
                    }

                    ClawGrab.setPosition(CLAW_RELEASE);
                    ClawTurn.setPosition(CLAW_DOWN_FLOOR_EXTEND);

                    /* ###########################################################
                    ########  Closing out loop by setting "done" to true  ########
                    ########################################################### */
                    done = true;
                }
            }

            if (gamepad2.dpad_left) {//This will be picking up specimen
                done = false;
                while (!done) {

                    ClawTurn.setPosition(CLAW_SPECIMEN_PICK_UP);
                    ClawGrab.setPosition(CLAW_RELEASE);

                    extendMotor.setTargetPosition(0);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(-1.0);

                    while (angMotor.getCurrentPosition() > ANGLE_SPECIMEN_FLOOR_PICK_UP) {
                        angMotor.setTargetPosition((int) ANGLE_SPECIMEN_FLOOR_PICK_UP);
                        angMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        angMotor.setPower(-1);
                    }
                    angMotor.setPower(0);

                    telemetry.addLine("Press X to Grab specimem and wait");



                    if (gamepad1.x) {
                        sleep(1000);
                        while (angMotor.getCurrentPosition() < ANGLE_ARM_READY_TO_CLIP) {
                            angMotor.setPower(1);
                        }
                    }
                    telemetry.clearAll();

                    /* ###########################################################
                    ########  Closing out loop by setting "done" to true  ########
                    ########################################################### */
                    done = true;

                }
            }

            if (gamepad2.dpad_down) {//This will be clipping specimen on FIRST BAR
                done = false;
                while (!done) {

                    ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
                    ClawGrab.setPosition(CLAW_GRAB);

                    extendMotor.setTargetPosition(0);
                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extendMotor.setPower(-1.0);

                    while (angMotor.getCurrentPosition() > ANGLE_ARM_READY_TO_CLIP) {//will only happen if arm is in zero position
                        angMotor.setPower(-1);
                    }
                    angMotor.setPower(0);
                    while (angMotor.getCurrentPosition() < ANGLE_ARM_READY_TO_CLIP) {//will only happen if arm is less than zero position
                        angMotor.setPower(1);
                    }
                    angMotor.setPower(0);

                    telemetry.addLine("PLAYER 2 PRESS           A          WHEN READY TO CLIP");

                    if (gamepad2.a) {
                        while (angMotor.getCurrentPosition() > ANGLE_ARM_DONE_CLIPPING) {
                            angMotor.setPower(-1);
                        }
                    }

                    /* ###########################################################
                    ########  Closing out loop by setting "done" to true  ########
                    ########################################################### */
                    done = true;

                }
            }

//            if (gamepad2.dpad_down) {//This will be clipping specimen on HIGH BAR
//                done = false;
//                while (!done) {
//
//                    ClawTurn.setPosition(CLAW_CLIPPING_POSITION);
//                    ClawGrab.setPosition(CLAW_GRAB);
//
//                    extendMotor.setTargetPosition(0);
//                    extendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    extendMotor.setPower(-1.0);
//
//                    while (angMotor.getCurrentPosition() > ANGLE_ARM_READY_TO_CLIP) {//will only happen if arm is in zero position
//                        angMotor.setPower(-1);
//                    }
//                    angMotor.setPower(0);
//                    while (angMotor.getCurrentPosition() < ANGLE_ARM_READY_TO_CLIP) {//will only happen if arm is less than zero position
//                        angMotor.setPower(1);
//                    }
//                    angMotor.setPower(0);
//
//                    telemetry.addLine("PLAYER 2 PRESS           A          WHEN READY TO CLIP");
//
//                    if (gamepad2.a) {
//                        while (angMotor.getCurrentPosition() > ANGLE_ARM_DONE_CLIPPING) {
//                            angMotor.setPower(-1);
//                        }
//                    }
//
//                    /* ###########################################################
//                    ########  Closing out loop by setting "done" to true  ########
//                    ########################################################### */
//                    done = true;
//
//                }
//            }



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

            if (gamepad1.x) {
                telemetry.addData("In gamepad1.x:", ClawGrab.getPosition());
                telemetry.update();
                ClawGrab.setPosition(CLAW_GRAB); // Close claw
            } else if (gamepad1.y) {
                telemetry.addData("In gamepad1.y:", ClawGrab.getPosition());
                telemetry.update();
                ClawGrab.setPosition(CLAW_RELEASE); // Open claw
            }

        }
    }
}



