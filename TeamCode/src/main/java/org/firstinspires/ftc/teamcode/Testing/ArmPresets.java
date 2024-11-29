package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name= "ArmPresets")
public class ArmPresets extends LinearOpMode {
    public DcMotorEx extendMotor;

    public DcMotorEx angMotor;

    public Servo ClawClamp, ClawTurn;

    @Override
    public void runOpMode() throws InterruptedException {

        ClawClamp = hardwareMap.get(Servo.class, "ClawClamp");
        ClawTurn = hardwareMap.get(Servo.class, "ClawTurn");
        angMotor = hardwareMap.get(DcMotorEx.class, "AngleMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "ExtendMotor");


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

        //Claw Stuff
        //[Goes Here]

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
            _______________________________________________________________________________________
            _______________________________________________________________________________________
             */

        }
    }
}
