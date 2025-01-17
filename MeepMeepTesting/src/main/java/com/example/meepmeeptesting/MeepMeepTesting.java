package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action clip1 = myBot.getDrive().actionBuilder(new Pose2d(11.5, -61, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -38), Math.toRadians(90))
                .build();

        Action clip1Hang = myBot.getDrive().actionBuilder(new Pose2d(0, -38, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -34.5), Math.toRadians(90))
                .build();

        Action clip2 = myBot.getDrive().actionBuilder(new Pose2d(0, -34.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(35, -40), Math.toRadians(90))
                .lineToY(-10)
                .splineTo(new Vector2d(45, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(45, -55))
                .lineToY(-47)
                .splineTo(new Vector2d(36, -45), Math.toRadians(180))
                .strafeTo(new Vector2d(36,-58))
                .strafeTo(new Vector2d(36, -55)) //Approach wall
                //.waitSeconds(1) //Grab clip
                .build();

        Action clip2Hang = myBot.getDrive().actionBuilder(new Pose2d(36, -55, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(0, -50), Math.toRadians(-90))
                .lineToY(-38)
                .build();

        Action clip2HangComplete = myBot.getDrive().actionBuilder(new Pose2d(0, -38, Math.toRadians(90)))
                .lineToY(-36)
                .build();

        Action clip3 = myBot.getDrive().actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(36, -45), Math.toRadians(90))
                .strafeTo(new Vector2d(36, -58))
                .strafeTo(new Vector2d(36, -55)) //Approach wall
                .build();

        myBot.runAction(  new SequentialAction(
                //clawServo.closeClaw(),
                clip1,
                //clawPivot.moveToPosition(0.2711), //Originally 0.3011
                //platformSlide.moveToPosition(2100), //Hanging first clip
                clip1Hang,
                //platformSlide.moveToPosition(900),
                //clawServo.openClaw(),

                clip2,
                //Grab clip from wall
                clip2Hang,
                clip2HangComplete,

                clip3,

                clip2Hang,
                clip2HangComplete
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}