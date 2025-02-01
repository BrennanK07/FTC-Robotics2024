
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
/*
@Autonomous(name = "ClipStates", group = "Autonomous")
public class StateClips extends LinearOpMode {
    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDimensions(18, 18)
            .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15) //kV 60, kA 60
            .build();



    Pose2d startPose = new Pose2d(11.8, -61.7, Math.toRadians(-90));  // Starting position

    TrajectoryActionBuilder specimen1 = myBot.getDrive().actionBuilder(startPose)
            .lineToY(-34);

    TrajectoryActionBuilder pushSamples = myBot.getDrive().actionBuilder(new Pose2d(11.8, -34, Math.toRadians(-90)))
            .setTangent(Math.toRadians(0))
            .lineToX(36)
            .setTangent(Math.toRadians(90))
            .lineToY(-10)
            .setTangent(Math.toRadians(0))
            .lineToXLinearHeading(48, Math.toRadians(90))
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
            .lineToX(35)
            .setTangent(Math.toRadians(90))
            .lineToY(-61.7);

    TrajectoryActionBuilder clipSpecimen2 = myBot.getDrive().actionBuilder(new Pose2d(35, -61.7, Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(10, -34), Math.toRadians(-90));

    TrajectoryActionBuilder grabSpecimen3 = myBot.getDrive().actionBuilder(new Pose2d(10, -34, Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(35, -61.7), Math.toRadians(90));

    TrajectoryActionBuilder clipSpecimen3 = myBot.getDrive().actionBuilder(new Pose2d(35, -61.7, Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(5, -34), Math.toRadians(-90));

    TrajectoryActionBuilder grabSpecimen4 = myBot.getDrive().actionBuilder(new Pose2d(5, -34, Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(35, -61.7), Math.toRadians(90));

    TrajectoryActionBuilder clipSpecimen4 = myBot.getDrive().actionBuilder(new Pose2d(35, -61.7, Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(-90));

    TrajectoryActionBuilder grabSpecimen5 = myBot.getDrive().actionBuilder(new Pose2d(0, -34, Math.toRadians(-90)))
            .strafeToLinearHeading(new Vector2d(35, -61.7), Math.toRadians(90));

    TrajectoryActionBuilder clipSpecimen5 = myBot.getDrive().actionBuilder(new Pose2d(35, -61.7, Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(-5, -34), Math.toRadians(-90));

    TrajectoryActionBuilder parkInObservationZone = myBot.getDrive().actionBuilder(new Pose2d(-5, -34, Math.toRadians(90)))
            .strafeToLinearHeading(new Vector2d(-5, -34), Math.toRadians(-90));


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
                                //Grab with claw
                                ),
                                new ParallelAction(
            clipSpecimen2.build()
    //Increase slide and pivot to bar
                ),
                        new SequentialAction(
            //Decrease slide and piot bar(clip)
            //Release claw
            grabSpecimen3.build(),
                        clipSpecimen3.build(),

                                grabSpecimen4.build(),
                                clipSpecimen4.build(),

                                grabSpecimen5.build(),
                                clipSpecimen5.build()
                                )
                                ));
}
*/
