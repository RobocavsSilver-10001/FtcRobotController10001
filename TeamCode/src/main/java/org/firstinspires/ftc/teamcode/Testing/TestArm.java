
package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name= "TestArm")
public class TestArm extends LinearOpMode {

    public DcMotorEx fl, fr, bl, br;
    public DcMotorEx extendMotorFront, extendMotorBack;
    public static int en = 50;
    DcMotorEx angMotorLeft, angMotorRight;

    public void runOpMode() throws InterruptedException {

        //Configuration for driver hub
        fl = hardwareMap.get(DcMotorEx.class,"FrontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRight");
        bl = hardwareMap.get(DcMotorEx.class,"BackLeft");
        br = hardwareMap.get(DcMotorEx.class,"BackRight");

        extendMotorFront = hardwareMap.get(DcMotorEx.class, "ExtendMotorFront");
        extendMotorBack = hardwareMap.get(DcMotorEx.class, "ExtendMotorBack");

        angMotorLeft = hardwareMap.get(DcMotorEx.class, "AngleMotorLeft");
        angMotorRight = hardwareMap.get(DcMotorEx.class, "AngleMotorRight");

        angMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        angMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        angMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        extendMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        extendMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        extendMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

        angMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Debug the current starting position values
        telemetry.addData("Starting Arm Position-Left Motor is ", angMotorLeft.getCurrentPosition());
        telemetry.addData("Starting Arm Position-Right Motor is ", angMotorRight.getCurrentPosition());
        telemetry.update();
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


            //slow mode
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


            //Controlling arm angle
            if (gamepad1.a) {
                //encoder
                telemetry.addData("Here 1 In the Gamepad A ", angMotorLeft.getCurrentPosition());
                telemetry.addData("Here 2 In the Gamepad A ", angMotorRight.getCurrentPosition());
                telemetry.update();
                moveArmToPositionUp(700);
                /*
                // Arm Position should be set to 816
                angMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                angMotorLeft.setTargetPosition(3);
                angMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //movement
                while (angMotorLeft.getCurrentPosition() < angMotorLeft.getTargetPosition()) {
                    angMotorLeft.setPower(0.5);
                    angMotorRight.setPower(0.5);
                }

                 */
            } else if (gamepad1.b) {
                telemetry.addData("Here 1 In the Gamepad B ", angMotorLeft.getCurrentPosition());
                telemetry.addData("Here 2 In the Gamepad B ", angMotorRight.getCurrentPosition());
                telemetry.update();
                moveArmToPositionDown(-215);
                /*
                angMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                angMotorLeft.setTargetPosition(5);
                angMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                while (angMotorLeft.getCurrentPosition() > angMotorLeft.getTargetPosition()) {
                    angMotorLeft.setPower(-0.5);
                    angMotorRight.setPower(-0.5);
                }

                 */
            }
            if (gamepad1.dpad_right) {
                extendMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extendMotorFront.setTargetPosition(5);
                extendMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                System.out.println("Here 1: Extend curr pos: " + extendMotorFront.getCurrentPosition());

                while (extendMotorFront.getCurrentPosition() < extendMotorFront.getTargetPosition()) {
                    System.out.println("Here 2: Extend curr pos: " + extendMotorFront.getCurrentPosition());
                    extendMotorFront.setPower(1);
                    extendMotorBack.setPower(1);
                }
            }
        }
    }
    private void moveArmToPositionUp(int targetPosition) {
        angMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        angMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        int velocity = 500;
        if (angMotorRight.getCurrentPosition() < targetPosition) {
            telemetry.addData("Here 20: current position ", angMotorLeft.getCurrentPosition());
            telemetry.addData("Here 20: Arm Position-Right Motor is ", angMotorRight.getCurrentPosition());
            telemetry.addData("Here 20: targetposition ", targetPosition);
            telemetry.update();

            angMotorLeft.setTargetPosition(targetPosition);
            angMotorRight.setTargetPosition(targetPosition);
            angMotorRight.setVelocity(velocity);
            angMotorLeft.setVelocity(velocity);
            //angMotorLeft.setPower(100); // Set power as needed
            //angMotorRight.setPower(100); // Set power as needed
            while (opModeIsActive() && angMotorLeft.isBusy() && angMotorRight.isBusy()) {
                velocity = velocity*(angMotorLeft.getCurrentPosition()/targetPosition);
                // Wait until the motor reaches the target position
                telemetry.addData("Arm Position-Left Motor is ", angMotorLeft.getCurrentPosition());
                telemetry.addData("Arm Position-Right Motor is ", angMotorRight.getCurrentPosition());
                telemetry.addData("Current Velocity is ", velocity);
                telemetry.update();

                angMotorRight.setVelocity(velocity);
                angMotorLeft.setVelocity(velocity);
            }
        }
        angMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        angMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        angMotorLeft.setPower(0.4); // Stop the motor
        angMotorRight.setPower(0.4); // Stop the motor
        //velocity = 30;
        //angMotorRight.setVelocity(velocity);
        //angMotorLeft.setVelocity(velocity);
    }
    private void moveArmToPositionDown(int targetPosition) {
        int velocity = 1000;
        angMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        angMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        if (angMotorRight.getCurrentPosition() > targetPosition) {
            angMotorLeft.setTargetPosition(targetPosition);
            angMotorRight.setTargetPosition(targetPosition);
            angMotorLeft.setPower(1); // Set power as needed
            angMotorRight.setPower(1); // Set power as needed
            while (opModeIsActive() && angMotorLeft.isBusy() && angMotorRight.isBusy()) {
                // Wait until the motor reaches the target position
                telemetry.addData("Arm Position-Left Motor is ", angMotorLeft.getCurrentPosition());
                telemetry.addData("Arm Position-Right Motor is ", angMotorRight.getCurrentPosition());
                telemetry.update();
            }
        }
        angMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        angMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //angMotorLeft.setPower(0.2); // Stop the motor
        //angMotorRight.setPower(0.2); // Stop the motor
        velocity = 50;
        angMotorRight.setVelocity(velocity);
        angMotorLeft.setVelocity(velocity);
    }
}


