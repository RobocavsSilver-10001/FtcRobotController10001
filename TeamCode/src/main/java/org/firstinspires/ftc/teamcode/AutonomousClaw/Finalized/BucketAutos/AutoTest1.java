/*package org.firstinspires.ftc.teamcode.AutonomousClaw.Finalized.BucketAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "DUB")
public class AdvancedRedBucket extends LinearOpMode {

    public DcMotorEx angMotor, extendMotor;
    public Servo ClawGrab, ClawTurn;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8, -60, Math.toRadians(90)));

        ClawGrab = hardwareMap.servo.get("ClawGrab");
        ClawTurn = hardwareMap.servo.get("ClawTurn");
        angMotor = hardwareMap.dcMotor.get("AngleMotor");
        extendMotor = hardwareMap.dcMotor.get("ExtendMotor");

        angMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Run the actions in parallel and sequential order
        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-8, -60, Math.toRadians(90)))
                        .stopAndAdd(new ParallelAction(
                                new ClawRelease(ClawGrab, .389),
                                new ClawAngle(ClawTurn, 0.33),
                                new ArmAngle(angMotor, 2890)
                        ))
                        .splineToConstantHeading(new Vector2d(-10 , -35.95), Math.toRadians(90))
                        .stopAndAdd(new ArmExtension(extendMotor, 860))
                        .waitSeconds(1)
                        .stopAndAdd(new ClawRelease(ClawGrab, .465))
                        .waitSeconds(1)
                        .lineToYConstantHeading(-50)

                        // Second ParallelAction
                        .stopAndAdd(new ParallelAction(
                                new ArmExtension(extendMotor, 0),
                                new ArmAngle(angMotor, -2615),
                                new ClawAngle(ClawTurn, .33)
                        ))
                        .splineToConstantHeading(new Vector2d(-46.25 , -39), Math.toRadians(90))
                        .waitSeconds(.5)
                        .stopAndAdd(new ArmExtension(extendMotor, 295))
                        .stopAndAdd(new ClawRelease(ClawGrab, .389))
                        .stopAndAdd(new ArmAngle(angMotor, 600))
                        .turn(2.45)
                        .stopAndAdd(new ArmAngle(angMotor, 3800))
                        .waitSeconds(.1)
                        .stopAndAdd(new ArmExtension(extendMotor, 2100))
                        .waitSeconds(.1)
                        .lineToX(-49)
                        .stopAndAdd(new ClawRelease(ClawGrab, .465))
                        .strafeToLinearHeading(new Vector2d(-50, -46), Math.toRadians(90))
                        .stopAndAdd(new ArmExtension(extendMotor, 0))
                        .stopAndAdd(new ArmAngle(angMotor, 0))
                        .build()
        ));
    }

    public class ClawRelease implements Action {
        Servo ClawGrab;
        double GrabPosition;

        public ClawRelease(Servo CG, double GP) {
            this.ClawGrab = CG;
            this.GrabPosition = GP;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ClawGrab.setPosition(GrabPosition);
            return false;
        }
    }

    public class ClawAngle implements Action {
        Servo ClawTurn;
        double TurnPosition;

        public ClawAngle(Servo CT, double TP) {
            this.ClawTurn = CT;
            this.TurnPosition = TP;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ClawTurn.setPosition(TurnPosition);
            return false;
        }
    }

    public class ArmAngle implements Action {
        DcMotor angMotor;
        int anglePosition;

        public ArmAngle(DcMotor aG, int aP) {
            this.angMotor = aG;
            this.anglePosition = aP;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            angMotor.setTargetPosition(anglePosition);
            angMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            angMotor.setPower(1);
            return angMotor.isBusy();
        }
    }

    public class ArmExtension implements Action {
        DcMotor extendMotor;
        int extendPosition;

        public ArmExtension(DcMotor eM, int eP) {
            this.extendMotor = eM;
            this.extendPosition = eP;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendMotor.setTargetPosition(extendPosition);
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendMotor.setPower(1);
            return extendMotor.isBusy();
        }
    }
}
*/