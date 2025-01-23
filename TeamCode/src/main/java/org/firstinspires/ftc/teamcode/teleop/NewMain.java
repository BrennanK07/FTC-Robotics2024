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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //Linear slides
        DcMotorEx leftClimbSlide = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightClimbSlide = hardwareMap.get(DcMotorEx.class, "rightFront");

        int lastPosition = 0;

        class PID{
            double kP = 1.0;
            double kD = 0.005;
            double kI = 0.05;

            double prevPos;

            double proprtional;
            double derivative;
            double integral;

            public double power;

            public PID(double currentPos){
                prevPos = currentPos;
            }

            public void updatePID(double currentPos, double targetPos, double deltaTime){
                proprtional = targetPos - currentPos;
                integral += currentPos;

                derivative = (currentPos - prevPos) / deltaTime;

                prevPos = currentPos;

                calculatePID();
            }

            public void calculatePID(){
                power = (kP * proprtional) + (kD * derivative) + (kI * integral);
            }
        }

        //Servos
        //Servo bucketSwing = hardwareMap.get(Servo.class, "bucketSwing");
        //Servo clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        //Servo clawGrab = hardwareMap.get(Servo.class, "clawGrab");

        //CRServos
        //CRServo bucketIntake = hardwareMap.get(CRServo.class, "bucketIntake");


        //Initialize motors with encoders before starting
        //bucketSlide.setTargetPosition(bucketSlide.getCurrentPosition());
        //bucketSlideTargetPos = bucketSlide.getCurrentPosition();
        //bucketSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftClimbSlide.setTargetPosition(0);
        leftClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimbSlide.setTargetPosition(0);
        rightClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PID leftClimbPID = new PID(leftClimbSlide.getCurrentPosition());
        PID rightClimbPID = new PID(rightClimbSlide.getCurrentPosition());

        int verticalSlideTargetPos = 0;
        double verticalSlideMaxSpeed = 2796.04;

        //Init motors / servos

        waitForStart();

        while (opModeIsActive()) {
            updateDeltaTime();

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
            leftClimbPID.updatePID(leftClimbSlide.getCurrentPosition(), lastPosition, deltaTime);
            rightClimbPID.updatePID(rightClimbSlide.getCurrentPosition(), lastPosition, deltaTime);

            if(Math.abs(gamepad2.right_stick_y) < 0.2){
                //Hold position
                leftClimbSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightClimbSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftClimbSlide.setTargetPosition(lastPosition);
                rightClimbSlide.setTargetPosition(lastPosition);

                if(lastPosition < -100) {
                    leftClimbSlide.setPower(leftClimbPID.power);
                    rightClimbSlide.setPower(rightClimbPID.power);
                }else{
                    leftClimbSlide.setPower(0);
                    rightClimbSlide.setPower(0);
                }
            }else{
                //Hold position
                leftClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftClimbSlide.setPower(gamepad2.right_stick_y);
                rightClimbSlide.setPower(gamepad2.right_stick_y);

                lastPosition = leftClimbSlide.getCurrentPosition();

                if(lastPosition < -3200){
                    lastPosition = -3200;
                }

                if(lastPosition > 0){
                    lastPosition = 0;
                }
            }

            telemetry.addData("Vertical Slide Target Position", verticalSlideTargetPos);

            telemetry.addData("Vertical Slide Position (L / R)", leftClimbSlide.getCurrentPosition() + " / " + rightClimbSlide.getCurrentPosition());

            //Macros


            //Telemetry
            drive.updatePoseEstimate();

            telemetry.addData("c2 Right Stick", gamepad2.right_stick_x + ", " + gamepad2.right_stick_y);

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