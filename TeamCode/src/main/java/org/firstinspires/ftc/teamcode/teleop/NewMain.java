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
        DcMotorEx leftClimbSlide = hardwareMap.get(DcMotorEx.class, "leftVerticalSlide");
        DcMotorEx rightClimbSlide = hardwareMap.get(DcMotorEx.class, "rightVerticalSlide");

        leftClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftClimbSlide.setTargetPosition(0);
        //leftClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimbSlide.setTargetPosition(0);
        //rightClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightClimbSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDTuner leftClimbPID = new PIDTuner(leftClimbSlide.getCurrentPosition());
        PIDTuner rightClimbPID = new PIDTuner(rightClimbSlide.getCurrentPosition());

        DcMotorEx intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
        //intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setTargetPosition(0);
        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double verticalSlideTargetPos = 0;
        //int lastPosition = 0;

        final double MAX_SPEED_MULTIPLIER = 0.9;
        final double VERT_SLIDE_MAX_SPEED = 2796.04 * MAX_SPEED_MULTIPLIER;

        //Servo leftIntakePivot = hardwareMap.get(Servo.class, "a");
        Servo rightIntakePivot = hardwareMap.get(Servo.class, "b");
        Servo clawRotator = hardwareMap.get(Servo.class, "c");
        Servo claw = hardwareMap.get(Servo.class, "d");

        CRServo liftThing = hardwareMap.get(CRServo.class, "a");

        Vector2 leftStick1 = new Vector2();
        Vector2 rightStick1 = new Vector2();

        boolean intakeExtended = false;

        //Init motors / servos

        waitForStart();

        while (opModeIsActive()) {
            updateDeltaTime();

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
            //leftClimbPID.updatePID(leftClimbSlide.getCurrentPosition(), (int) verticalSlideTargetPos, deltaTime);
            //rightClimbPID.updatePID(rightClimbSlide.getCurrentPosition(), (int) verticalSlideTargetPos, deltaTime);

            //Master-Slave approach (Weaker one is the master)
            //leftClimbPID.updatePID(leftClimbSlide.getCurrentPosition(), verticalSlideTargetPos, deltaTime);
            //rightClimbPID.updatePID(rightClimbSlide.getCurrentPosition(), leftClimbSlide.getCurrentPosition(), deltaTime);

            if(Math.abs(gamepad2.right_stick_y) < 0.1){ //Hold Position Mode
                //Hold position

                if(leftClimbSlide.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                    leftClimbSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightClimbSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    leftClimbSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightClimbSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                //leftClimbSlide.setTargetPosition((int) verticalSlideTargetPos);
                //rightClimbSlide.setTargetPosition((int) verticalSlideTargetPos);

                leftClimbSlide.setTargetPosition((int) verticalSlideTargetPos);
                rightClimbSlide.setTargetPosition((int) verticalSlideTargetPos);

                if(verticalSlideTargetPos > 100) {
                    leftClimbSlide.setPower(0.1);
                    rightClimbSlide.setPower(0.1);
                }else{
                    leftClimbSlide.setPower(0);
                    rightClimbSlide.setPower(0);
                }
            }else{ //Run Mode
                if(leftClimbSlide.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) || leftClimbSlide.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)) {
                    leftClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                leftClimbSlide.setPower(-gamepad2.right_stick_y);
                rightClimbSlide.setPower(-gamepad2.right_stick_y);

                //lastPosition = leftClimbSlide.getCurrentPosition();

                //lastPosition = clamp(lastPosition, -4000, 0);

                verticalSlideTargetPos = leftClimbSlide.getCurrentPosition();

                rightClimbSlide.setTargetPosition((int) verticalSlideTargetPos);
                verticalSlideTargetPos = clamp(verticalSlideTargetPos, 0, 3800);
            }

            telemetry.addData("VerticalSlides/Target Position", verticalSlideTargetPos);
            telemetry.addData("VerticalSlides/Positions", leftClimbSlide.getCurrentPosition() + " " + rightClimbSlide.getCurrentPosition());

            telemetry.addData("IntakeSlide/Target Position", intakeSlide.getTargetPosition());
            telemetry.addData("IntakeSlide/Actual Position", intakeSlide.getCurrentPosition());

            //Servos
            liftThing.setPower(gamepad2.left_stick_y);

            //Macros
            if(gamepad2.a && !controllerSysB.getPressedButtons().contains("A")){
                //Extends slide and lowers to ground
                if(!intakeExtended) {
                    intakeSlide.setTargetPosition(100);

                    intakeExtended = true;
                }else{
                    intakeSlide.setTargetPosition(0);

                    intakeExtended = false;
                }
            }

            if(gamepad2.b && !controllerSysB.getPressedButtons().contains("B")){
                //Retracts slide and transfers to vertical slide

            }

            if(gamepad2.x && !controllerSysB.getPressedButtons().contains("X")){
                //takes grabbed sample and raises it for bucket
            }

            if(gamepad2.y && !controllerSysB.getPressedButtons().contains("Y")){
                //takes grabbed specimen and raises it for top rung
            }

            //Zero motors
            if(gamepad1.x && gamepad1.y && gamepad1.left_bumper){
                leftClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftClimbSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightClimbSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftClimbSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightClimbSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

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

            //Updating Statistics
            controllerSysA.updatePressedButtons();
            controllerSysB.updatePressedButtons();
        }
    }
}