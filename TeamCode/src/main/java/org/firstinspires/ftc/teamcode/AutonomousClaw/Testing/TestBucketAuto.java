
package org.firstinspires.ftc.teamcode.AutonomousClaw.Testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DefaultRRFiles.MecanumDrive;

@Autonomous(name = "BucketAutoOne")
public class TestBucketAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8, -60, Math.toRadians(90)));
        Servo ClawGrab = hardwareMap.servo.get("ClawGrab");
        Servo ClawTurn = hardwareMap.servo.get("ClawTurn");
        DcMotor angMotor = hardwareMap.dcMotor.get("AngleMotor");
        DcMotor extendMotor = hardwareMap.dcMotor.get("ExtendMotor");


        angMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(-8, -60, Math.toRadians(90)))
                        //to score specimen
                        .strafeToConstantHeading(new Vector2d(-5, -35))
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, 0))
                        .stopAndAdd(new TestBucketAuto.ClawAngle(ClawTurn, .2))
                        .stopAndAdd(new TestBucketAuto.ArmAngle(angMotor, 2650))
                        .waitSeconds(.5)
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 1320))
                        .waitSeconds(1.5)
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, 0.55))
                        .waitSeconds(.5)
                        .lineToYConstantHeading(-32)
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 0))
                        .stopAndAdd(new TestBucketAuto.ArmAngle(angMotor, -1375))
                        .stopAndAdd(new TestBucketAuto.ClawAngle(ClawTurn, .5))
                        .waitSeconds(.5)
                        .lineToYConstantHeading(-35)
                        //score first sample
                        //pick up sample
                        .splineToConstantHeading(new Vector2d(-48, -56), Math.toRadians(90))
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 1750))
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, .65))
                        .waitSeconds(1.5)
                        //.stopAndAdd(new TestBucketAuto.ClawAngle(ClawTurn, .5))
                        .stopAndAdd(new TestBucketAuto.ArmAngle(angMotor, 3800))
                        .waitSeconds(2.5)
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 2900))
                        .waitSeconds(1.5)
                        //score
                        .turnTo(Math.toRadians(215))
                        .waitSeconds(.2)
                        .lineToX(-50)
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, .55))
                        .waitSeconds(.5)
                        //score second sample
                        //pick up sample
                        .strafeToLinearHeading(new Vector2d(-58, -50), Math.toRadians(90))
                        .waitSeconds(.5)
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 2000))
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, .65))
                        .waitSeconds(.75)
                        .stopAndAdd(new TestBucketAuto.ClawAngle(ClawTurn, .5))
                        .stopAndAdd(new TestBucketAuto.ArmAngle(angMotor, 3800))
                        .waitSeconds(1.5)
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 2900))
                        .waitSeconds(1.5)
                        //score sample
                        .strafeToLinearHeading(new Vector2d(-48, -56), Math.toRadians(215))
                        .waitSeconds(.5)
                        .strafeToLinearHeading(new Vector2d(-49, -57), Math.toRadians(215))
                        .lineToX(-50)
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, .55))
                        .waitSeconds(.5)
                        //score third sample
                        //pick up sample
                        .strafeToLinearHeading(new Vector2d(-61, -50), Math.toRadians(100))
                        .waitSeconds(.5)
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 2000))
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, .65))
                        .waitSeconds(.75)
                        .stopAndAdd(new TestBucketAuto.ClawAngle(ClawTurn, .5))
                        .stopAndAdd(new TestBucketAuto.ArmAngle(angMotor, 3800))
                        .waitSeconds(1.5)
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 2900))
                        .waitSeconds(1.5)
                        //score sample
                        .strafeToLinearHeading(new Vector2d(-48, -56), Math.toRadians(215))
                        .strafeToLinearHeading(new Vector2d(-49, -57), Math.toRadians(215))
                        .lineToX(-50)
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, .55))
                        .waitSeconds(.5)
                        //park
                        .strafeToLinearHeading(new Vector2d(-25, 0), Math.toRadians(0))
                        .waitSeconds(.5)
                        .stopAndAdd(new TestBucketAuto.ArmExtension(extendMotor, 0))
                        .stopAndAdd(new TestBucketAuto.ArmAngle(angMotor, 0))
                        .stopAndAdd(new TestBucketAuto.ClawRelease(ClawGrab, .65))
                        .stopAndAdd(new TestBucketAuto.ClawAngle(ClawTurn, .5))
                        .build()));
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

            extendMotor.setPower(.5);
            return extendMotor.isBusy();
        }
    }
}