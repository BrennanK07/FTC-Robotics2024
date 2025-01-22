package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(18, 18)
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15) //kV 60, kA 60
                .build();

        Pose2d startPose = new Pose2d(11.8, -61.7, Math.toRadians(90));  // Starting position

        TrajectoryActionBuilder specimen1 = myBot.getDrive().actionBuilder(startPose)
                .lineToY(-34);

        TrajectoryActionBuilder pushSamples = myBot.getDrive().actionBuilder(new Pose2d(11.8, -34, Math.toRadians(90)))
                .setTangent(Math.toRadians(0))
                .lineToX(34)
                .setTangent(Math.toRadians(90))
                .lineToY(-10)
                .setTangent(Math.toRadians(0))
                .lineToX(48)
                //Push first spikezone sample
                .setTangent(Math.toRadians(90))
                .lineToY(-55)
                .lineToY(-10)
                .setTangent(Math.toRadians(0))
                .lineToX(57)
                //Push second spikezone sample
                .setTangent(Math.toRadians(90))
                .lineToY(-55)
                .lineToY(-10)
                .setTangent(Math.toRadians(0))
                .lineToX(61)
                //Push third spikezone sample
                .setTangent(Math.toRadians(90))
                .lineToY(-55)
                .lineToY(-45);

        TrajectoryActionBuilder grabSpecimen2 = myBot.getDrive().actionBuilder(new Pose2d(61, -45, Math.toRadians(90)))
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(35, Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .lineToY(-61.7);


        myBot.runAction(new SequentialAction(
                new ParallelAction(
                        //Raise slide
                        //Raise claw
                        specimen1.build()
                ),
                new SequentialAction(
                        pushSamples.build(),
                        //Drop slide / claw
                        grabSpecimen2.build()
                )
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}