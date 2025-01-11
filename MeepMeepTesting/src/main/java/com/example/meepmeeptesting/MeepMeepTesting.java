package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, -60, Math.toRadians(90)))
                //to score specimen
                .strafeToConstantHeading(new Vector2d(-5, -35))
                .waitSeconds(.5)
                .waitSeconds(1.5)
                .waitSeconds(.5)
                .lineToYConstantHeading(-32)
                .waitSeconds(.5)
                .lineToYConstantHeading(-35)
                //score first sample
                //pick up sample
                .splineToConstantHeading(new Vector2d(-48, -56), Math.toRadians(90))
                .waitSeconds(1.5)
                //.stopAndAdd(new TestBucketAuto.ClawAngle(ClawTurn, .5))
                .waitSeconds(2.5)
                .waitSeconds(1.5)
                //score
                .turnTo(Math.toRadians(215))
                .waitSeconds(.2)
                .lineToX(-50)
                .waitSeconds(.5)
                //score second sample
                //pick up sample
                .strafeToLinearHeading(new Vector2d(-58, -50), Math.toRadians(90))
                .waitSeconds(.5)
                .waitSeconds(.75)
                .waitSeconds(1.5)
                .waitSeconds(1.5)
                //score sample
                .strafeToLinearHeading(new Vector2d(-48, -56), Math.toRadians(215))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-49, -57), Math.toRadians(215))
                .lineToX(-50)
                .waitSeconds(.5)
                //score third sample
                //pick up sample
                .strafeToLinearHeading(new Vector2d(-61, -50), Math.toRadians(100))
                .waitSeconds(.5)
                .waitSeconds(.75)
                .waitSeconds(1.5)
                .waitSeconds(1.5)
                //score sample
                .strafeToLinearHeading(new Vector2d(-48, -56), Math.toRadians(215))
                .strafeToLinearHeading(new Vector2d(-49, -57), Math.toRadians(215))
                .lineToX(-50)
                .waitSeconds(.5)
                //park
                .strafeToLinearHeading(new Vector2d(-25, 0), Math.toRadians(0))
                .waitSeconds(.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
