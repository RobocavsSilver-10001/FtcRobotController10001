package org.firstinspires.ftc.teamcode.Autonomous.Testing.PathTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DefaultRRFiles.MecanumDrive;

@Config
@Autonomous(name = "TestSpecimenRed", group = "AutoTesting")
public class TestSpecimenRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8, -60,Math.toRadians(90) ));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(8,-60, Math.toRadians(90)))
                        .splineToConstantHeading(new Vector2d(8, -35), Math.toRadians(90))
                        .waitSeconds(1)
                        //
                        .lineToYConstantHeading(-55)
                        .waitSeconds(1)
                        .strafeToConstantHeading(new Vector2d(50, -55))
                        .build()
        );

    }

}
