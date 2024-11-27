package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PrototypeServoTest")
public class PrototypeServoTest extends LinearOpMode{
    Servo[] servos = new Servo[5];;
    int currentServo = 0;
    double[] servoPos = new double[5];

    final double SERVO_INCREMENT = 0.001;

    boolean isPressed = false;
    boolean wasUsingControlStick = false;

    @Override
    public void runOpMode() {
        servos[0] = hardwareMap.get(Servo.class, "a");
        servos[1] = hardwareMap.get(Servo.class, "b");
        servos[2] = hardwareMap.get(Servo.class, "c");
        servos[3] = hardwareMap.get(Servo.class, "d");
        servos[4] = hardwareMap.get(Servo.class, "e");

        waitForStart();

        while(opModeIsActive()){
            //servoPos = servos[currentServo].getPosition();

            /*
            if(gamepad1.a){
                servoPos[currentServo] += SERVO_INCREMENT;
            }else if(gamepad1.b){
                servoPos[currentServo] -= SERVO_INCREMENT;
            }*/

            //Looping through servos
            if(gamepad1.y){
                if(!isPressed) {
                    currentServo++;

                    if (currentServo > 4) {
                        currentServo = 0;
                    }

                    if (currentServo < 0) {
                        currentServo = 4;
                    }

                    isPressed = true;
                }
            }else{
                isPressed = false;
            }

            //Setting position
            if(gamepad1.left_stick_y != 0) {
                servoPos[currentServo] = gamepad1.left_stick_y;
                wasUsingControlStick = true;
            }
            else{
                if(wasUsingControlStick){
                    servoPos[currentServo] = 0;

                    wasUsingControlStick = false;
                }

                if(gamepad1.dpad_up) {
                    servoPos[currentServo] += SERVO_INCREMENT;
                }else if(gamepad1.dpad_down){
                    servoPos[currentServo] -= SERVO_INCREMENT;
                }

                servoPos[currentServo] = Math.max(0, Math.min(1, servoPos[currentServo]));
            }

            for(int i = 0; i < 5; i++) {
                servos[i].setPosition(servoPos[i]);
            }

            telemetry.addData("Servo Position", servoPos[currentServo]);
            telemetry.addData("Current Servo", currentServo);
            telemetry.update();
        }
    }
}
