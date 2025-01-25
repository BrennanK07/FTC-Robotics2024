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
import org.firstinspires.ftc.teamcode.tools.ControllerInputSystem;
import org.firstinspires.ftc.teamcode.tools.PIDTuner;
import org.firstinspires.ftc.teamcode.tools.Util22156;

import static org.firstinspires.ftc.teamcode.tools.Util22156.*;
import org.firstinspires.ftc.teamcode.tools.Util22156.Vector2;

import java.util.Vector;

@TeleOp(name = "Competition TeleOp")

public class NewMain extends LinearOpMode{
    final double TURBO_MOTOR_SPEED = 0.9;
    boolean SHOW_DEBUG_VALUES = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, -34, Math.toRadians(90)));

        ControllerInputSystem controllerSysA = new ControllerInputSystem(gamepad1);
        ControllerInputSystem controllerSysB = new ControllerInputSystem(gamepad2);

        //Linear slides
        DcMotorEx leftClimbSlide = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightClimbSlide = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftClimbSlide.setTargetPosition(0);
        leftClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimbSlide.setTargetPosition(0);
        rightClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDTuner leftClimbPID = new PIDTuner(leftClimbSlide.getCurrentPosition());
        PIDTuner rightClimbPID = new PIDTuner(rightClimbSlide.getCurrentPosition());

        double verticalSlideTargetPos = 0;
        int lastPosition = 0;

        final double MAX_SPEED_MULTIPLIER = 0.5;
        final double VERT_SLIDE_MAX_SPEED = 2796.04 * MAX_SPEED_MULTIPLIER;

        Vector2 leftStick1 = new Vector2();
        Vector2 rightStick1 = new Vector2();


        //Init motors / servos

        waitForStart();

        while (opModeIsActive()) {
            updateDeltaTime();

            controllerSysA.updatePressedButtons();
            controllerSysB.updatePressedButtons();

            //Drive motors thru roadrunner
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
            //Maybe set targetPositions to equal the other?
            leftClimbPID.updatePID(leftClimbSlide.getCurrentPosition(), verticalSlideTargetPos, deltaTime);
            rightClimbPID.updatePID(rightClimbSlide.getCurrentPosition(), verticalSlideTargetPos, deltaTime);

            //Master-Slave approach (Weaker one is the master)
            //leftClimbPID.updatePID(leftClimbSlide.getCurrentPosition(), verticalSlideTargetPos, deltaTime);
            //rightClimbPID.updatePID(rightClimbSlide.getCurrentPosition(), leftClimbSlide.getCurrentPosition(), deltaTime);

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
                leftClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftClimbSlide.setPower(leftClimbPID.power);
                rightClimbSlide.setPower(rightClimbPID.power);

                lastPosition = leftClimbSlide.getCurrentPosition();

                lastPosition = clamp(lastPosition, -3100, 0);

                verticalSlideTargetPos += gamepad2.right_stick_y * VERT_SLIDE_MAX_SPEED * deltaTime;
                verticalSlideTargetPos = clamp(verticalSlideTargetPos, -3100, 0);
            }

            telemetry.addData("VerticalSlides/Target Position", verticalSlideTargetPos);
            telemetry.addData("VerticalSlides/Positions", leftClimbSlide.getCurrentPosition() + " " + rightClimbSlide.getCurrentPosition());

            //Macros


            //Telemetry
            drive.updatePoseEstimate();

            if(gamepad1.left_bumper && gamepad1.right_bumper){
                if(!controllerSysA.getPressedButtons().contains("Left Bumper")){ //Stops multi activation on button hold
                    SHOW_DEBUG_VALUES = !SHOW_DEBUG_VALUES;
                }
            }

            if(SHOW_DEBUG_VALUES) {
                debugControllers(telemetry, controllerSysA, controllerSysB);

                leftClimbPID.debugPID(telemetry, "Left Climb PID");
                rightClimbPID.debugPID(telemetry, "Right Climb PID");
            }

            telemetry.addData("General/deltaTime (s)", deltaTime);

            telemetry.addData("RoadrunnerPos/x", drive.pose.position.x);
            telemetry.addData("RoadrunnerPos/y", drive.pose.position.y);
            telemetry.addData("RoadrunnerPos/heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("RoadrunnerPos/heading (rad)", drive.pose.heading.toDouble());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}