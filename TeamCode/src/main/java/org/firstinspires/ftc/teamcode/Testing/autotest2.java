package org.firstinspires.ftc.teamcode.Testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DefaultRRFiles.MecanumDrive;

@Autonomous(name = "DUB2")
public class autotest2 extends LinearOpMode {

    public DcMotorEx angMotor, extendMotor;
    public Servo ClawGrab, ClawTurn;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8, -60, Math.toRadians(90)));

        ClawGrab = hardwareMap.servo.get("ClawGrab");
        ClawTurn = hardwareMap.servo.get("ClawTurn");
        angMotor = hardwareMap.get(DcMotorEx.class, "AngleMotor");
        extendMotor = hardwareMap.get(DcMotorEx.class, "ExtendMotor");

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
                                new ArmAngle(angMotor, 2715)
                        ))
                        .splineToConstantHeading(new Vector2d(-10 , -36), Math.toRadians(90))
                        .stopAndAdd(new ArmExtension(extendMotor, 815))
                        .waitSeconds(.1)
                        .stopAndAdd(new ClawRelease(ClawGrab, .465))
                        .waitSeconds(.1)
                        .lineToYConstantHeading(-50)

                        // Second ParallelAction
                        .stopAndAdd(new ParallelAction(
                                new ArmExtension(extendMotor, 0),
                                new ArmAngle(angMotor, -2625),
                                new ClawAngle(ClawTurn, .33)
                        ))
                        .splineToConstantHeading(new Vector2d(-46.25 , -39.75), Math.toRadians(90))
                        .waitSeconds(.1)
                        .stopAndAdd(new ArmExtension(extendMotor, 300))
                        .stopAndAdd(new ClawRelease(ClawGrab, .389))
                        .stopAndAdd(new ArmAngle(angMotor, 3800))
                        .waitSeconds(1.65)
                        .stopAndAdd(new ArmExtension(extendMotor, 2100))
                        .waitSeconds(.1)
                        .turn(2.545)
                        .waitSeconds(.1)
                        .lineToX(-50.9)
                        .waitSeconds(.1)
                        .stopAndAdd(new ClawRelease(ClawGrab, .465))
                        .waitSeconds(.1)
                        .strafeToLinearHeading(new Vector2d(-54.55, -40.5), Math.toRadians(94))
                        .stopAndAdd(new ParallelAction(
                                new ArmExtension(extendMotor, 500),
                                new ArmAngle(angMotor, -1994)
                        ))
                        .waitSeconds(2.25)
                        .stopAndAdd(new ClawRelease(ClawGrab, .389))
                        .stopAndAdd(new ParallelAction(
                                new ArmExtension(extendMotor, 500),
                                new ArmAngle(angMotor, 3800)
                        ))
                        .turn(2.68)
                        .stopAndAdd(new ArmExtension(extendMotor, 2100))
                        .waitSeconds(.1)
                        .stopAndAdd(new ClawRelease(ClawGrab, .465))



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
