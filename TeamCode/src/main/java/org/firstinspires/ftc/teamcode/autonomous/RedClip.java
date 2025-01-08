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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RedClip", group = "Autonomous")
public class RedClip extends LinearOpMode {
    public class PlatformSlide {
        private final DcMotor platformSlide;

        public PlatformSlide(HardwareMap hardwareMap){
            platformSlide = hardwareMap.get(DcMotor.class, "platformSlide");

            platformSlide.setTargetPosition(platformSlide.getCurrentPosition());
            platformSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            platformSlide.setPower(1.0);
        }

        public class MoveToPosition implements Action{
            private int targetPosition = 0;

            public MoveToPosition(int position){
                targetPosition = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                platformSlide.setTargetPosition(targetPosition);
                while(platformSlide.getCurrentPosition() != targetPosition){
                    //Run until motor is at the right height
                }
                return false;
            }
        }

        public Action moveToPosition(int position){
            return new MoveToPosition(position);
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-11.5, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //Linear slides
        DcMotor bucketSlide = hardwareMap.get(DcMotor.class, "bucketSlide"); //Max extension 2380
        PlatformSlide platformSlide = new PlatformSlide(hardwareMap); //Max extension 3900

        //DcMotor climbSlideFront = hardwareMap.get(DcMotor.class, "motorName");
        //DcMotor climbSlideBack = hardwareMap.get(DcMotor.class, "motorName");

        //Servos
        Servo bucketSwing = hardwareMap.get(Servo.class, "bucketSwing");
        Servo clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        Servo clawGrab = hardwareMap.get(Servo.class, "clawGrab");

        //CRServos
        CRServo bucketIntake = hardwareMap.get(CRServo.class, "bucketIntake");

        //Initialize motors with encoders before starting
        bucketSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketSlide.setTargetPosition(bucketSlide.getCurrentPosition());

        Action clip1 = drive.actionBuilder(new Pose2d(11.5, -61, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -36), Math.toRadians(90))
                .waitSeconds(1)
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(36, -36))
                .build();

        Action clip2 = drive.actionBuilder(new Pose2d(36, -36, Math.toRadians(0)))
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

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        //clip1,
                        platformSlide.moveToPosition(2500) //Hanging first clip
                        //clip2
                )
        );
    }
}
