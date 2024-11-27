package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PrototypeServoTest")
public class PrototypeServoTest extends LinearOpMode{
    Servo[] servos = new Servo[5];;
    int currentServo = 0;
    double servoPos = 0;

    final double SERVO_INCREMENT = 0.01;

    @Override
    public void runOpMode() {
        servos[0] = hardwareMap.get(Servo.class, "deviceName");
        servos[1] = hardwareMap.get(Servo.class, "deviceName");
        servos[2] = hardwareMap.get(Servo.class, "deviceName");
        servos[3] = hardwareMap.get(Servo.class, "deviceName");
        servos[4] = hardwareMap.get(Servo.class, "deviceName");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                servoPos += SERVO_INCREMENT;
            }else if(gamepad1.b){
                servoPos -= SERVO_INCREMENT;
            }

            //Looping through menus
            if(gamepad1.y){
                currentServo++;

                if(currentServo > 4){
                    currentServo = 0;
                }

                if(currentServo < 0){
                    currentServo = 4;
                }
            }

            servoPos = Math.max(0.0, Math.min(1.0, servoPos));

            servos[currentServo].setPosition(servoPos);

            telemetry.addData("Servo Position", servoPos);
            telemetry.addData("Current Servo", servos[currentServo].getDeviceName());
            telemetry.update();
        }
    }
}
