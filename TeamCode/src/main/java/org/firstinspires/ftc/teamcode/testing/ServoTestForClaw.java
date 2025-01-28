package org.firstinspires.ftc.teamcode.testing;

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
        double[] powers = new double[4];

        int activeServo = 0;

        ControllerInputSystem controllerSys = new ControllerInputSystem(gamepad1);

        for(Servo s : servos){
            s.setPosition(s.getPosition());
        }

        for(int i = 0; i < 4; i++){
            powers[i] = servos[i].getPosition();
        }

        waitForStart();

        while(opModeIsActive()){
            updateDeltaTime();
           controllerSys.updatePressedButtons();

           if(gamepad1.a && !controllerSys.getPressedButtons().contains("a")){
               activeServo++;

               if(activeServo > 3){
                   activeServo = 0;
               }
           }

           for(int i = 0; i < 4; i++) {
               if(i == activeServo) {
                   servos[i].setPosition(powers[i] + (5 * deltaTime * gamepad1.left_stick_x));
               }else{
                   servos[i].setPosition(powers[i]);
               }
           }

           telemetry.addData("current active servo", activeServo);
           telemetry.addData("servo position", servos[activeServo].getPosition());
           telemetry.update();
        }
    }
}
