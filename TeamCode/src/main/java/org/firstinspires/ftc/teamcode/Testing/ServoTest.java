package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonomousClaw.Testing.TestBucketAuto;

@Config
@TeleOp(name = "AxonServoTest")
public class ServoTest extends LinearOpMode {

    public Servo clawTurn, clawGrab;

    public static double pos = 0.0;
    final double CLAW_DOWN_FLOOR_EXTEND = 0.56;
    final double CLAW_SCORE_TOP_BUCKET = 0.55;
    final double CLAW_HOME_POSITION = 0.55;
    final double CLAW_SPECIMEN_PICK_UP = 0.55;
    final double CLAW_SPECIMEN_WALL_PICK_UP = 0.73;
    final double CLAW_CLIPPING_POSITION = 0.5594;
    final double CLAW_GRAB = 0.7;      // Fully closed
    final double CLAW_RELEASE = 0.62;  // Fully open

    @Override
    public void runOpMode() throws InterruptedException {

        clawTurn = hardwareMap.get(Servo.class, "ClawTurn");
        clawGrab = hardwareMap.get(Servo.class, "ClawGrab");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad2.a) {
                clawTurn.setPosition(pos);
            }
            else if (gamepad2.b) {
                clawTurn.setPosition(0);
            } else if (gamepad2.x) {
                clawTurn.setPosition(0);
            }

//            if (gamepad2.a) {
//            ClawGrab.setPosition(0.402);
//            } else if (gamepad2.b) {
//            ClawGrab.setPosition(0.5);
//            }
        }

    }

}
