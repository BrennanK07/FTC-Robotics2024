package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tools.ControllerInputSystem;

//@TeleOp(name = "ServoPosTest")
public class ServoPosTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo a = hardwareMap.get(Servo.class, "a");
        Servo b = hardwareMap.get(Servo.class, "b");
        Servo c = hardwareMap.get(Servo.class, "c");
        Servo d = hardwareMap.get(Servo.class, "d");

        Servo[] servos = {a, b, c, d};

        waitForStart();

        while(opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                telemetry.addData("servo " + i, servos[i].getPosition());
            }

            telemetry.update();
        }
    }
}
