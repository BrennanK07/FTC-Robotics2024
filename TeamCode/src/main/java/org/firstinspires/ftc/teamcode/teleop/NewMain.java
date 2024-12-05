package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Competition TeleOp")

public class Main extends LinearOpMode{
    //Initalizing Variables / Constants
    private final DcMotor[] driveMotor = new DcMotor[4]; //[fl, fr, bl, br]
    private final double[] driveMotorPower = new double[4]; //Power to each motor


    
    @Override
    public void runOpMode(){
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            telemetry.update();
        }
    }
}