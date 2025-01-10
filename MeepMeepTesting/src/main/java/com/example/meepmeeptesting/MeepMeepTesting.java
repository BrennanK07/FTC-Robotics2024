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
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Action clip1 = myBot.getDrive().actionBuilder(new Pose2d(11.5, -61, Math.toRadians(90)))
                .splineTo(new Vector2d(11.5, -34.4), Math.toRadians(90))
                .build();

        Action clip2 = myBot.getDrive().actionBuilder(new Pose2d(11.5, -34.4, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(35, -40), Math.toRadians(90))
                .lineToY(-10)
                .splineTo(new Vector2d(45, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(45, -55))
                .lineToY(-47)
                .splineTo(new Vector2d(36, -45), Math.toRadians(180))
                .strafeTo(new Vector2d(36,-54.75))
                //.strafeTo(new Vector2d(36, -55)) //Approach wall
                //.waitSeconds(1) //Grab clip
                .build();

        Action clip2Hang = myBot.getDrive().actionBuilder(new Pose2d(36, -54.75, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(0, -50), Math.toRadians(-90))
                .lineToY(-35.5)
                .build();

        Action clip3 = myBot.getDrive().actionBuilder(new Pose2d(0, -35.5, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(35, -40), Math.toRadians(90))
                .lineToY(-10)
                .splineTo(new Vector2d(57, -10), Math.toRadians(0))
                .strafeTo(new Vector2d(57, -55))
                //.lineToY(-47)
                //.splineTo(new Vector2d(36, -45), Math.toRadians(180))
                //.strafeTo(new Vector2d(36,-54.75))
                //.strafeTo(new Vector2d(36, -55)) //Approach wall
                //.waitSeconds(1) //Grab clip
                .build();

        Action waitForClip = myBot.getDrive().actionBuilder(new Pose2d(36, -55, Math.toRadians(-90)))
                .waitSeconds(1)
                .build();

        myBot.runAction(  new SequentialAction(
                //clawServo.closeClaw(),
                //clawPivot.moveToPosition(0.5),
                clip1,
                //clip1Hang,
                //platformSlide.moveToPosition(2050), //Hanging first clip
                //clawPivot.moveToPosition(0.2711), //Originally 0.2711
                //platformSlide.moveToPosition(850),
                //clawServo.openClaw(),

                clip2,
                //Grab clip from wall
                //clawPivot.moveToPosition(0.195),
                //platformSlide.moveToPosition(0),
                waitForClip,
                //clawServo.closeClaw(),
                //clawPivot.moveToPosition(0.5),
                //platformSlide.moveToPosition(850),

                clip2Hang,
                //clip2HangComplete,
                //clawPivot.moveToPosition(0.2711), //Originally 0.3011
                //platformSlide.moveToPosition(2050), //Hanging second clip
                //clawPivot.moveToPosition(0.2711),
                //platformSlide.moveToPosition(850),
                //clawServo.openClaw(),

                clip3
                //Grab clip from wall
                //platformSlide.moveToPosition(0),
                //clawPivot.moveToPosition(0.195),
                //clawServo.closeClaw(),
                //clawPivot.moveToPosition(0.2711),
                //platformSlide.moveToPosition(850),

        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}