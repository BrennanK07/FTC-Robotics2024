package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.Vector;

@TeleOp(name = "Competition TeleOp")

public class NewMain extends LinearOpMode{
    double oldUnixTimestamp = System.nanoTime() * 1e-9;
    double currentUnixTimestamp = System.nanoTime() * 1e-9;
    double deltaTime = System.nanoTime() * 1e-9;

    final double TURBO_MOTOR_SPEED = 0.9;

    public class Joystick{
        public double x;
        public double y;

        public Joystick(double x, double y){
            this.x = x;
            this.y = y;
        }

        public Joystick(){
            this(0, 0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //Linear slides
        DcMotorEx leftClimbSlide = hardwareMap.get(DcMotorEx.class, "leftClimbSlide");
        DcMotorEx rightClimbSlide = hardwareMap.get(DcMotorEx.class, "rightClimbSlide");

        //Servos
        Servo bucketSwing = hardwareMap.get(Servo.class, "bucketSwing");
        Servo clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        Servo clawGrab = hardwareMap.get(Servo.class, "clawGrab");

        //CRServos
        CRServo bucketIntake = hardwareMap.get(CRServo.class, "bucketIntake");


        //Initialize motors with encoders before starting
        //bucketSlide.setTargetPosition(bucketSlide.getCurrentPosition());
        //bucketSlideTargetPos = bucketSlide.getCurrentPosition();
        //bucketSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftClimbSlide.setDirection(DcMotorEx.Direction.FORWARD);

        rightClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimbSlide.setDirection(DcMotorEx.Direction.FORWARD);

        //Init motors / servos

        //Controller Inputs
        Joystick leftStick2 = new Joystick();
        Joystick rightStick2 = new Joystick();

        waitForStart();

        while (opModeIsActive()) {
            updateDeltaTime();

            //Poll inputs
            leftStick2 = new Joystick(gamepad2.left_stick_x, gamepad2.left_stick_y);
            rightStick2 = new Joystick(gamepad2.right_stick_x, gamepad2.right_stick_y);


            //Drive motors
            if(gamepad1.right_bumper) { //Fast speed
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * TURBO_MOTOR_SPEED, //Left-right
                                -gamepad1.left_stick_x * TURBO_MOTOR_SPEED //Up-down
                        ),
                        -gamepad1.right_stick_x * TURBO_MOTOR_SPEED //Heading adjust
                ));
            }else{
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * 0.5, //Left-right
                                -gamepad1.left_stick_x * 0.5 //Up-down
                        ),
                        -gamepad1.right_stick_x * 0.3 //Heading adjust
                ));
            }

            //Slides and Servos
            if(Math.abs(leftStick2.y) > 0.2){
                leftClimbSlide.setPower(leftStick2.y);
                rightClimbSlide.setPower(leftStick2.y);
            }else{
                leftClimbSlide.setPower(0);
                rightClimbSlide.setPower(0);
            }

            //Macros


            //Telemetry
            drive.updatePoseEstimate();

            telemetry.addData("deltaTime (s)", deltaTime);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private void updateDeltaTime(){
        oldUnixTimestamp = currentUnixTimestamp;
        currentUnixTimestamp = System.nanoTime()* 1e-9;

        deltaTime = currentUnixTimestamp - oldUnixTimestamp;
    }
}