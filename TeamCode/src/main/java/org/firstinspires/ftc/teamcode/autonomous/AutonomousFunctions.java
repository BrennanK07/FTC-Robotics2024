package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonomousFunctions {
    /*
        Action redClip = drive.actionBuilder(new Pose2d(11.5, -61, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -36), Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(36, -36))
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
                .build();

        Action redBucket = drive.actionBuilder(new Pose2d(-11.5, -61, Math.toRadians(90)))
                .splineTo(new Vector2d(-50, -50), Math.toRadians(225))
                .waitSeconds(3)
                .splineTo(new Vector2d(-25, 0), Math.toRadians(0))
                .waitSeconds(3)
                .lineToX(-35)
                .splineTo(new Vector2d(-50, -50), Math.toRadians(45))
                .waitSeconds(3)
                .build();

        Action testPath = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(48, 48), Math.PI / 2)
                .waitSeconds(30)
                .endTrajectory()
                .build();
     */
}
