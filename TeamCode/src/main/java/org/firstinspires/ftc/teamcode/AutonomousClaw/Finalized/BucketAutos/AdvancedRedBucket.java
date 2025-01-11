
package org.firstinspires.ftc.teamcode.AutonomousClaw.Finalized.BucketAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DefaultRRFiles.MecanumDrive;

@Autonomous(name = "DUB")
public class AdvancedRedBucket extends LinearOpMode {
    //

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
                        .stopAndAdd(new ClawRelease(ClawGrab, .389))
                        .stopAndAdd(new ClawAngle(ClawTurn, 0.33))
                        .stopAndAdd(new ArmAngle(angMotor, 3200))
                        .splineToConstantHeading(new Vector2d(-5 , -36.5), Math.toRadians(90))
                        .stopAndAdd(new ArmExtension(extendMotor, 650))
                        .waitSeconds(1)
                        .stopAndAdd(new ClawRelease(ClawGrab, .465))
                        .waitSeconds(1)
                        .lineToYConstantHeading(-50)
                        .stopAndAdd(new ArmExtension(extendMotor, 0))
                        .stopAndAdd(new ArmAngle(angMotor, -2640))
                        .stopAndAdd(new ClawAngle(ClawTurn, .33))
                        .splineToConstantHeading(new Vector2d(-44.5 , -40), Math.toRadians(90))
                        .waitSeconds(.5)
                        .stopAndAdd(new ArmExtension(extendMotor, 268))
                        .stopAndAdd(new ClawRelease(ClawGrab, .389))
                        .waitSeconds(.5)
                        .stopAndAdd(new ArmAngle(angMotor, 3800))
                        .waitSeconds(2)
                        .stopAndAdd(new ArmExtension(extendMotor, 2900))
                        .splineToConstantHeading(new Vector2d(-49.5 , -40), Math.toRadians(215))
                        /*.lineToX(-49)
                        .stopAndAdd(new ClawRelease(ClawGrab, .465))
                        .strafeToLinearHeading(new Vector2d(-50, -46), Math.toRadians(90))
                        .stopAndAdd(new ArmExtension(extendMotor, 0))
                        .stopAndAdd(new ArmAngle(angMotor, 0))*/
                        .build()));


        // OPENS THE CLAW .stopAndAdd(new ClawRelease(ClawGrab, 0.55)) */
        //arm angle
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
