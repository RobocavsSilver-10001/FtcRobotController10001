package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*Tick Data
        Extend Arm Out Max = 2500 Ticks
        Extend Arm In Max = 0 Ticks (Start code with Arm all the way in)

 */

@TeleOp(name= "ArmPresets")
public class ArmPresets extends LinearOpMode {
    public DcMotorEx extendMotor;

    public DcMotorEx angMotor;

    public Servo ClawGrab;
    public Servo ClawTurn;

    final double CLAW_DOWN_FLOOR_EXTEND = 0.5;
    final double CLAW_SCORE_TOP_BUCKET = 0.8;
    final double CLAW_UP_POSITION = -1.0;
    final double CLAW_ZERO_POSITION = 0.0;


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
        angMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Extend Arm Stuff
        extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //Claw Stuff
        ClawTurn.setPosition(CLAW_UP_POSITION);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

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
            if (gamepad1.left_trigger > 0.5) {
                extendMotor.setPower(1);

                telemetry.addData("Extension Pos:", extendMotor.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.right_trigger > 0.5) {
                extendMotor.setPower(-1);

                telemetry.addData("Extension Pos:", extendMotor.getCurrentPosition());
                telemetry.update();
            } else {
                extendMotor.setPower(0);
            }
            /*
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */
            /*  Claw Turn
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */
            if (gamepad2.a) {
                ClawTurn.setPosition(CLAW_SCORE_TOP_BUCKET);
                telemetry.addData("Claw Turn Pos:", ClawTurn.getPosition());
            } else if (gamepad2.b){
                ClawTurn.setPosition(CLAW_DOWN_FLOOR_EXTEND);
                telemetry.addData("Claw Turn Pos:", ClawTurn.getPosition());
            } else {
                ClawTurn.setPosition(CLAW_ZERO_POSITION);
            }
            /*
            _______________________________________________________________________________________
            _______________________________________________________________________________________
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */


        }
    }
}
