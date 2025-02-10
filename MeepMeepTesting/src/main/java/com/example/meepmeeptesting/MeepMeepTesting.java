
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.1)
                .build(); //our current values, 100, 125,

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, 58, Math.toRadians(90)))
                        .setTangent(Math.toRadians(-90))
                        .splineTo(new Vector2d(0,30), Math.toRadians(-90)) // drive to bar
                        .strafeTo(new Vector2d(0, 25)) // ram bar
                        .strafeTo(new Vector2d(0, 35)) // back up from bar
                        .splineToLinearHeading(new Pose2d(-36, 22, Math.toRadians(-90)), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-36, 8), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(-45, 2, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-45, 50, Math.toRadians(-90)), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-48, 10), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-55, 10, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-60, 10), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-65, 10, Math.toRadians(-90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-65, 50), Math.toRadians(90))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}