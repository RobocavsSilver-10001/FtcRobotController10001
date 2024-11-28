/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;


@Config
@TeleOp(name="RaghavTest1", group="Linear OpMode")
public class RaghavTest1 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx extendMotorFront, extendMotorBack;
    public DcMotorEx angMotorLeft, angMotorRight;
    public static int armMax= 700;
    public static int armMin= -500;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        //extendMotorFront = hardwareMap.get(DcMotorEx.class, "ExtendMotorFront");
        //extendMotorBack = hardwareMap.get(DcMotorEx.class, "ExtendMotorBack");

        //angMotorLeft = hardwareMap.get(DcMotorEx.class, "AngleMotorLeft");
        angMotorRight = hardwareMap.get(DcMotorEx.class, "AngleMotor");


        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    //    angMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        angMotorRight.setDirection(DcMotorEx.Direction.FORWARD);

        //extendMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
       // extendMotorBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        angMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        angMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        extendMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        extendMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //    angMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        angMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        angMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
  //      angMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        angMotorLeft.setVelocity(500);
//        angMotorRight.setVelocity(500);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

            if (gamepad1.a) {
                if (angMotorRight.getCurrentPosition() < armMax) {
                 //   angMotorLeft.setPower(1.0);
                    angMotorRight.setPower(1.0);
                    angMotorRight.setTargetPosition(armMax);
               //     angMotorLeft.setTargetPosition(armMax);
                 //   angMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    angMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                 //   angMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    angMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                //    angMotorLeft.setPower(0.0);
                    //angMotorRight.setPower(0.0);
                }
            } else {
//                angMotorLeft.setPower(0);
                angMotorRight.setPower(0);
            }
            if (gamepad1.b) {
               // angMotorLeft.setPower(-0.5);
                angMotorRight.setPower(-1.0);
              //  angMotorLeft.setTargetPosition(armMin);
                angMotorRight.setTargetPosition(armMin);
              //  angMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                angMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            } else {
              //  angMotorLeft.setPower(0);
               //anMotorRight.setPower(0);
            }
        }
    }
}
