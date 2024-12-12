package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Competition TeleOp")

public class Main extends LinearOpMode{
    double oldUnixTimestamp = System.nanoTime() * 1e-9;
    double currentUnixTimestamp = System.nanoTime() * 1e-9;
    double deltaTime = System.nanoTime() * 1e-9;

    static double ENCODER_SPEED = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //Linear slides
        DcMotor bucketSlide = hardwareMap.get(DcMotor.class, "motorName");
        DcMotor platformSlide = hardwareMap.get(DcMotor.class, "motorName");

        DcMotor climbSlideFront = hardwareMap.get(DcMotor.class, "motorName");
        DcMotor climbSlideBack = hardwareMap.get(DcMotor.class, "motorName");

        //Servos
        Servo bucketSwing = hardwareMap.get(Servo.class, "motorName");

        //CRServos
        CRServo bucketIntake = hardwareMap.get(CRServo.class, "motorName");


        //Initialize motors with encoders before starting
        bucketSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        while (opModeIsActive()) {
            updateDeltaTime();

            //Drive motors
            drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                    -gamepad1.left_stick_y, //Left-right
                    -gamepad1.left_stick_x //Up-down
                ), 
                -gamepad1.right_stick_x //Heading adjust
            ));

            //Linear slides
            bucketSlide.setTargetPosition(
                bucketSlide.getCurrentPostition() + (gamepad2.left_stick_y * deltaTime * MOTOR_SPEED)
            );

            //Telemetry
            drive.updatePoseEstimate();

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