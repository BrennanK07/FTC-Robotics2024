package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action clip1 = myBot.getDrive().actionBuilder(new Pose2d(11.5, -61, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -36), Math.toRadians(90))
                .waitSeconds(1)
                .build();

        Action clip2 = myBot.getDrive().actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(35, -50), Math.toRadians(90))
                .lineToY(-61)
                .waitSeconds(1) //Grab clip
                .build();

        Action clip2Hang = myBot.getDrive().actionBuilder(new Pose2d(36, -61, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(0, -50), Math.toRadians(-90))
                .lineToY(-36)
                .build();

        /*
        Action pushClips = drive.actionBuilder(new Pose2d(0, -36, Math.toRadians(0)))
                .turn(Math.toRadians(-90))
                .lineToX(36)
                .turn(Math.toRadians(90))
                .lineToY(-10)
                .strafeTo(new Vector2d(45, -15)) //Sample 1
                .strafeTo(new Vector2d(45, -55))
                .strafeTo(new Vector2d(45, -10))
                .strafeTo(new Vector2d(55, -15)) //Sample 2
                .strafeTo(new Vector2d(55, -55))
                .strafeTo(new Vector2d(55, -10))
                .strafeTo(new Vector2d(61, -15)) //Sample 3
                .strafeTo(new Vector2d(61, -55))
                .strafeTo(new Vector2d(36, -61)) //Grabbing sample 1
                //Grab sample function
                .strafeTo(new Vector2d(0, -36))
                //Clip sample to top bar
                .strafeTo(new Vector2d(36, -61)) //Grabbing sample 2
                //Grab sample function
                .strafeTo(new Vector2d(0, -36))
                //Clip sample to top bar
                .strafeTo(new Vector2d(36, -61)) //Grabbing sample 3
                //Grab sample function
                .strafeTo(new Vector2d(0, -36))
                //Clip sample to top bar
                .strafeTo(new Vector2d(36, -61)) //Grabbing sample 4
                //Grab sample function
                .strafeTo(new Vector2d(0, -36))
                //Clip sample to top bar
                .build();*/ //Complete 5 clips route

        Action pushClips = myBot.getDrive().actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .turn(Math.toRadians(-90))
                .lineToX(36)
                .turn(Math.toRadians(90))
                .lineToY(-10)
                .strafeTo(new Vector2d(45, -15)) //Sample 1
                .strafeTo(new Vector2d(45, -55))
                .strafeTo(new Vector2d(45, -10))

                .splineTo(new Vector2d(36, -61), Math.toRadians(-90)) //Grabbing sample 1
                .build();

        Action clip3 = myBot.getDrive().actionBuilder(new Pose2d(36, -61, Math.toRadians(-90)))
                //Grab sample
                .lineToY(-50)
                .splineTo(new Vector2d(0, -50), Math.toRadians(-90))
                .lineToY(-36)
                .build();


        myBot.runAction((new SequentialAction(clip1, clip2, clip2Hang, pushClips, clip3)));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}