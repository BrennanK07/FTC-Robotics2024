package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PrototypeMotorTest")

public class PrototypeMotorTest extends LinearOpMode {
    public String configName = "worm_gear"; //INSERT MOTOR NAME IN CONFIG BETWEEN ""
    public String motorType = "dc"; //servo, or dc

    public Servo testServo;
    public DcMotor testMotor;

    @Override
    public void runOpMode(){
        //Init motor / servo if it exists
        if(!configName.isEmpty()){
            if(motorType.equals("servo")){
                testServo = hardwareMap.get(Servo.class, configName);
            }
            if(motorType.equals("dc")){
                testMotor = hardwareMap.get(DcMotor.class, configName);
            }
        }

        while(opModeIsActive()){
            if(motorType.equals("servo")){
                testServo.setPosition(testServo.getPosition() + gamepad1.left_stick_y);
            }

            if(motorType.equals("dc")){
                testMotor.setPower(gamepad1.left_stick_y);
            }

            telemetry.addData("Status", "Running");
            if(motorType.equals("servo")){
                telemetry.addData("Servo Position", testServo.getPosition());
            }

            telemetry.update();
        }
    }
}
