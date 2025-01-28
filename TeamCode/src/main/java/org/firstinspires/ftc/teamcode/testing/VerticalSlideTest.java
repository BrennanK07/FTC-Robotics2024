package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "VerticalSlideTest")
public class VerticalSlideTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Linear slides
        DcMotorEx leftClimbSlide = hardwareMap.get(DcMotorEx.class, "leftVerticalSlide");
        DcMotorEx rightClimbSlide = hardwareMap.get(DcMotorEx.class, "rightVerticalSlide");

        leftClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftClimbSlide.setTargetPosition(0);
        leftClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightClimbSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimbSlide.setTargetPosition(0);
        rightClimbSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightClimbSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightClimbSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            leftClimbSlide.setPower(gamepad2.right_stick_y);
            rightClimbSlide.setPower(gamepad2.right_stick_y);
        }
    }
}
