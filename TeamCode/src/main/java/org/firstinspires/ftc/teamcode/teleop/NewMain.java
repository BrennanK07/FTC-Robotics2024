package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name = "Competition TeleOp")

public class NewMain extends LinearOpMode{
    double oldUnixTimestamp = System.nanoTime() * 1e-9;
    double currentUnixTimestamp = System.nanoTime() * 1e-9;
    double deltaTime = System.nanoTime() * 1e-9;

    final double TURBO_MOTOR_SPEED = 0.9;
    final double ENCODER_SPEED = 2796.04;
    final double SWING_SERVO_SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //Linear slides
        DcMotor bucketSlide = hardwareMap.get(DcMotor.class, "bucketSlide"); //Max extension 2380
        DcMotor platformSlide = hardwareMap.get(DcMotor.class, "platformSlide"); //Max extension 3900
        int platformSlideTargetPos = 0;
        int bucketSlideTargetPos = 0;

        //DcMotor climbSlideFront = hardwareMap.get(DcMotor.class, "motorName");
        //DcMotor climbSlideBack = hardwareMap.get(DcMotor.class, "motorName");

        //Servos
        Servo bucketSwing = hardwareMap.get(Servo.class, "bucketSwing");
        Servo clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        Servo clawGrab = hardwareMap.get(Servo.class, "clawGrab");

        //CRServos
        CRServo bucketIntake = hardwareMap.get(CRServo.class, "bucketIntake");


        //Initialize motors with encoders before starting
        //bucketSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //bucketSlide.setTargetPosition(bucketSlide.getCurrentPosition());
        bucketSlide.setTargetPosition(bucketSlide.getCurrentPosition());
        bucketSlideTargetPos = bucketSlide.getCurrentPosition();
        bucketSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //bucketSlide.setTargetPosition(bucketSlide.getCurrentPosition());

        //platformSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //platformSlide.setTargetPosition(platformSlide.getCurrentPosition());
        platformSlide.setTargetPosition(bucketSlide.getCurrentPosition());
        platformSlideTargetPos = platformSlide.getCurrentPosition();
        platformSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //platformSlide.setTargetPosition(platformSlide.getCurrentPosition());
        //platformSlide.setTargetPosition(0);

        //Init motors / servos
        bucketSwing.setPosition(bucketSwing.getPosition());
        clawPivot.setPosition(clawPivot.getPosition());
        clawGrab.setPosition(clawGrab.getPosition());



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

            //Linear slides
            //Bucket slide
            /*
            if(-gamepad2.left_stick_x < 0) {
                bucketSlide.setTargetPosition(0);
            }else{
                bucketSlide.setTargetPosition(2380);
            }
            bucketSlide.setPower(-gamepad2.left_stick_x);*/

            bucketSlide.setPower(1.0);
            bucketSlideTargetPos += (int)(ENCODER_SPEED * deltaTime * -gamepad2.left_stick_x);

            if(bucketSlideTargetPos > 3500){
                bucketSlideTargetPos = 3500;
            }
            if(bucketSlideTargetPos < 0){
                bucketSlideTargetPos = 0;
            }

            bucketSlide.setTargetPosition(bucketSlideTargetPos);

            //Platform slide
            //platformSlide.setPower(1.0);
            platformSlideTargetPos += (int)(ENCODER_SPEED * deltaTime * -gamepad2.right_stick_y);

            if(platformSlideTargetPos > 3500){
                platformSlideTargetPos = 3500;
            }
            if(platformSlideTargetPos < 0){
                platformSlideTargetPos = 0;
            }

            if(platformSlideTargetPos < 100 && Math.abs(gamepad2.right_stick_y) < 0.1){
                platformSlide.setPower(0.0);
            }else{
                platformSlide.setPower(1.0);
                platformSlide.setTargetPosition(platformSlideTargetPos);
            }

            telemetry.addData("Platform Slide Position", platformSlide.getCurrentPosition());
            telemetry.addData("Platform Slide Target Position", platformSlideTargetPos);

            //Bucket Servo
            if(gamepad2.a) {
                bucketIntake.setPower(1);
            }else if(gamepad2.b){
                bucketIntake.setPower(-1);
            }else{
                bucketIntake.setPower(0);
            }

            //telemetry.addData("Intake trigger", gamepad2.right_trigger);
            //Min 0.5689 Level 0.2461 Max 0.2
            bucketSwing.setPosition(bucketSwing.getPosition() + (deltaTime * gamepad2.left_stick_y * SWING_SERVO_SPEED));

            if(bucketSwing.getPosition() > 0.5689){
                bucketSwing.setPosition(0.5689);
            }else if(bucketSwing.getPosition() < 0.1){
                bucketSwing.setPosition(0.1);
            }

            telemetry.addData("bucketSwing", bucketSwing.getPosition());

            //Claw Servos
            if(gamepad2.left_bumper){
                clawGrab.setPosition(0.9885); //originally 0.9925
            }else{
                clawGrab.setPosition(1);
            }
            //clawGrab.setPosition(1.0 - (gamepad2.left_trigger / 20));

            if(gamepad2.right_trigger > 0.2) {
                clawPivot.setPosition(clawPivot.getPosition() + (deltaTime * SWING_SERVO_SPEED * 0.5));

                if(clawPivot.getPosition() > 0.39){
                    clawPivot.setPosition(0.55);
                }
            }else if(gamepad2.left_trigger > 0.2){
                clawPivot.setPosition(clawPivot.getPosition() - (deltaTime * SWING_SERVO_SPEED * 0.5));

                if(clawPivot.getPosition() < 0){
                    clawPivot.setPosition(1);
                }
            }

            telemetry.addData("Claw Pivot Position", clawPivot.getPosition() /*+ (deltaTime * SWING_SERVO_SPEED)*/);

            //Macros


            //Telemetry
            drive.updatePoseEstimate();

            telemetry.addData("debug bucketSlide val", bucketSlide.getCurrentPosition() + (int)(-gamepad2.left_stick_y * deltaTime * ENCODER_SPEED));
            telemetry.addData("bucketSlide encoder position", bucketSlide.getCurrentPosition());

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