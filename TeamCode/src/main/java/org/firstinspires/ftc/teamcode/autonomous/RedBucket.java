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

//@Config
//@Autonomous(name = "RedBucket", group = "Autonomous")
public class RedBucket extends LinearOpMode {
    public Pose2d initialPose = new Pose2d(-11.5, -61, Math.toRadians(90));
    public MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

    @Override
    public void runOpMode() {
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        redBucket
                )
        );
    }

    public Action redBucket = drive.actionBuilder(new Pose2d(-11.5, -61, Math.toRadians(90)))
            .splineTo(new Vector2d(-50, -50), Math.toRadians(225))
            .waitSeconds(3)
            .splineTo(new Vector2d(-25, 0), Math.toRadians(0))
            .waitSeconds(3)
            .lineToX(-35)
            .splineTo(new Vector2d(-50, -50), Math.toRadians(45))
            .waitSeconds(3)
            .build();
}
