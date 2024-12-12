package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Competition TeleOp")

public class Main extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //Linear slides
        DcMotor bucketSlide = new DcMotor();
        DcMotor platformSlide = new DcMotor();

        DcMotor climbSlideFront = new DcMotor();
        DcMotor climbSlideBack = new DcMotor();

        //Servos
        Servo bucketSwing = new Servo();

        //CRServos
        CRServo bucketIntake = new CRServo();

        waitForStart();

        while (opModeIsActive()) {
            //Drive motors
            drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                    -gamepad1.left_stick_y, //Left-right
                    -gamepad1.left_stick_x //Up-down
                ), 
                -gamepad1.right_stick_x //Heading adjust
            ));

            //Linear slides
            

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
}