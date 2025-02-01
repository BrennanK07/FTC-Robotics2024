package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBuckets {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(18, 18)
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15) //kV 60, kA 60
                .build();



        Pose2d startPose = new Pose2d(-11.8, -61.7, Math.toRadians(-90));  // Starting position

        TrajectoryActionBuilder sample1 = myBot.getDrive().actionBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-53, -53), Math.toRadians(225));

        TrajectoryActionBuilder grabSample2 = myBot.getDrive().actionBuilder(new Pose2d(-53, -53, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-48.5, -40), Math.toRadians(90));

        TrajectoryActionBuilder scoreSample2 = myBot.getDrive().actionBuilder(new Pose2d(-48.5, -40, Math.toRadians(90)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45));

        TrajectoryActionBuilder grabSample3 = myBot.getDrive().actionBuilder(new Pose2d(-53, -53, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-57, -40), Math.toRadians(90));

        TrajectoryActionBuilder scoreSample3 = myBot.getDrive().actionBuilder(new Pose2d(-57, -40, Math.toRadians(90)))
                //.setReversed(true)
                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45));

        TrajectoryActionBuilder grabSample4 = myBot.getDrive().actionBuilder(new Pose2d(-53, -53, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-57, -26), Math.toRadians(180));

        TrajectoryActionBuilder scoreSample4 = myBot.getDrive().actionBuilder(new Pose2d(-57, -26, Math.toRadians(180)))
                //.setReversed(true)
                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45));


        myBot.runAction(new SequentialAction(
                new ParallelAction(
                        //Raise slide
                        //Raise claw
                        sample1.build()
                ),
                new SequentialAction(
                        grabSample2.build(),
                        scoreSample2.build(),

                        grabSample3.build(),
                        scoreSample3.build(),

                        grabSample4.build(),
                        scoreSample4.build()
                )
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}