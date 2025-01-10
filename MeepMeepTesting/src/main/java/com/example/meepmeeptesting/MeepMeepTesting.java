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
                .splineToConstantHeading(new Vector2d(-5 , -37), Math.toRadians(90))
                .waitSeconds(1)
                .lineToYConstantHeading(-50)
                .splineToConstantHeading(new Vector2d(-44.5 , -40), Math.toRadians(90))
                .waitSeconds(1.5)
                .turn(2.45)
                .lineToX(-55)
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-59, -52), Math.toRadians(90))
                .waitSeconds(.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
