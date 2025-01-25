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

//@Config
//@Autonomous(name = "RedClip", group = "Autonomous")
public class RedClip extends LinearOpMode {
    public static class PlatformSlide {
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
                platformSlide.setPower(0.5);
                platformSlide.setTargetPosition(targetPosition);

                while(Math.abs(platformSlide.getCurrentPosition() - targetPosition) > 10){
                    //wait for slide to reach height
                }

                return false;
            }
        }

        public Action moveToPosition(int position){
            return new MoveToPosition(position);
        }
    }

    public static class ClawServo {
        private final Servo clawServo;

        public ClawServo(HardwareMap hardwareMap){
            clawServo = hardwareMap.get(Servo.class, "clawGrab");
        }

        public class OpenClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawServo.setPosition(0.9885);

                //while(clawServo.getPosition() != 0.9885){
                    //wait for servo
                //}

                return false;
            }
        }

        public class CloseClaw implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawServo.setPosition(1.0);

                //while(clawServo.getPosition() != 1.0){
                    //wait for servo
                //}

                return false;
            }
        }

        public Action openClaw(){
            return new OpenClaw();
        }

        public Action closeClaw(){
            return new CloseClaw();
        }
    }

    public static class ClawPivot {
        private final Servo clawPivot;

        public ClawPivot(HardwareMap hardwareMap){
            clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        }

        public class MoveToPosition implements Action{
            double targetPosition;

            public MoveToPosition(double position){
                targetPosition = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                clawPivot.setPosition(targetPosition);

                while(clawPivot.getPosition() != targetPosition){
                    //wait for servo
                }

                return false;
            }
        }

        public Action moveToPosition(double position){
            return new MoveToPosition(position);
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.5, -61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //Linear slides
        DcMotor bucketSlide = hardwareMap.get(DcMotor.class, "bucketSlide"); //Max extension 2380
        PlatformSlide platformSlide = new PlatformSlide(hardwareMap); //Max extension 3900

        //DcMotor climbSlideFront = hardwareMap.get(DcMotor.class, "motorName");
        //DcMotor climbSlideBack = hardwareMap.get(DcMotor.class, "motorName");

        //Servos
        Servo bucketSwing = hardwareMap.get(Servo.class, "bucketSwing");
        ClawPivot clawPivot = new ClawPivot(hardwareMap);
        ClawServo clawServo = new ClawServo(hardwareMap);

        //CRServos
        CRServo bucketIntake = hardwareMap.get(CRServo.class, "bucketIntake");

        //Initialize motors with encoders before starting
        bucketSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketSlide.setTargetPosition(bucketSlide.getCurrentPosition());

        Action clip1 = drive.actionBuilder(new Pose2d(11.5, -61, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -38), Math.toRadians(90))
                .build();

        Action clip1Hang = drive.actionBuilder(new Pose2d(0, -38, Math.toRadians(90)))
                .splineTo(new Vector2d(0, -34.5), Math.toRadians(90))
                .build();

        Action clip2 = drive.actionBuilder(new Pose2d(0, -34.5, Math.toRadians(90)))
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

        Action clip2Hang = drive.actionBuilder(new Pose2d(36, -55, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(0, -50), Math.toRadians(-90))
                .lineToY(-38)
                .build();

        Action clip2HangComplete = drive.actionBuilder(new Pose2d(0, -38, Math.toRadians(90)))
                .lineToY(-36)
                .build();

        Action clip3 = drive.actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(36, -45), Math.toRadians(90))
                .strafeTo(new Vector2d(36, -58))
                .strafeTo(new Vector2d(36, -55)) //Approach wall
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        clawServo.closeClaw(),
                        clip1,
                        clawPivot.moveToPosition(0.2711), //Originally 0.3011
                        platformSlide.moveToPosition(2100), //Hanging first clip
                        clip1Hang,
                        platformSlide.moveToPosition(900),
                        clawServo.openClaw(),

                        clip2,
                        //Grab clip from wall
                        platformSlide.moveToPosition(0),
                        clawServo.closeClaw(),
                        platformSlide.moveToPosition(1000),

                        clip2Hang,
                        clawPivot.moveToPosition(0.2711), //Originally 0.3011
                        platformSlide.moveToPosition(2100), //Hanging second clip
                        clip2HangComplete,
                        platformSlide.moveToPosition(900),
                        clawServo.openClaw(),

                        clip3,

                        clip2Hang,
                        clawPivot.moveToPosition(0.2711), //Originally 0.3011
                        platformSlide.moveToPosition(2100), //Hanging second clip
                        clip2HangComplete,
                        platformSlide.moveToPosition(900),
                        clawServo.openClaw()
                )
        );
    }
}