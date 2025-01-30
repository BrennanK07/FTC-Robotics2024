package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.tools.Util22156.clamp;
import static org.firstinspires.ftc.teamcode.tools.Util22156.deltaTime;
import static org.firstinspires.ftc.teamcode.tools.Util22156.updateDeltaTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tools.ControllerInputSystem;

@TeleOp(name = "ServoClawValueDebugger")

public class ServoTestForClaw extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo a = hardwareMap.get(Servo.class, "a");
        Servo b = hardwareMap.get(Servo.class, "b");
        Servo c = hardwareMap.get(Servo.class, "c");
        Servo d = hardwareMap.get(Servo.class, "d");

        Servo[] servos = {a, b, c, d};
        double[] powers = {0.5, 0.5, 0.5, 0.5};

        int activeServo = 0;

        ControllerInputSystem controllerSys = new ControllerInputSystem(gamepad1);

        // Ensure servos stay at initial positions before the op mode starts
        for (int i = 0; i < 4; i++) {
            servos[i].setPosition(powers[i]);
        }

        waitForStart();

        while(opModeIsActive()){
            updateDeltaTime();

           if(gamepad1.a && !controllerSys.getPressedButtons().contains("A")){
               activeServo++;

               if(activeServo > 3){
                   activeServo = 0;
               }
           }

           for(int i = 0; i < 4; i++) {
               if(i == activeServo) {
                   servos[i].setPosition(powers[i] + (5 * deltaTime * gamepad1.left_stick_y));
                   powers[i] += (0.5 * deltaTime * gamepad1.left_stick_y);

                   powers[i] = clamp(powers[i], 0, 1);
               }else{
                   servos[i].setPosition(powers[i]);
               }
           }

            controllerSys.updatePressedButtons();

           telemetry.addData("current active servo", activeServo);

           for(int i = 0; i < 4; i++) {
               telemetry.addData("Servo Position " + i, servos[i].getPosition());
           }

            for(int i = 0; i < 4; i++) {
                telemetry.addData("Servo Intended Position " + i, powers[i]);
            }

           telemetry.update();
        }
    }
}
