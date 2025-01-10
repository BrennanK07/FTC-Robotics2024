package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Calibration")
public class MotorCalibration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Linear slides
        DcMotor bucketSlide = hardwareMap.get(DcMotor.class, "bucketSlide"); //Max extension 2380
        DcMotor platformSlide = hardwareMap.get(DcMotor.class, "platformSlide"); //Max extension 3900
        int platformSlideTargetPos = 0;

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
        bucketSlide.setTargetPosition(0);
        bucketSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        platformSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        platformSlide.setTargetPosition(0);
        platformSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Motors recalibrated", "Press Start to exit");
        telemetry.update();

        waitForStart();
    }
}
