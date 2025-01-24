package org.firstinspires.ftc.teamcode.tools;

/*
 * INSTRUCTIONS FOR USE (you're welcome chris):
 *
 * import static org.firstinspires.ftc.teamcode.tools.Util22156.*;
 *
 * Once you do this you should be able to just call for functions
 * ex:
 * clamp(a, b, c);
 *
 */

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Util22156 {
    //Clamps value to specified range
    public static int clamp(int val, int min, int max){
        if(val < min){
            return min;
        }

        if(val > max){
            return max;
        }

        return val;
    }

    public static double clamp(double val, double min, double max){
        if(val < min){
            return min;
        }

        if(val > max){
            return max;
        }

        return val;
    }

    //Debug controller input outputting
    public static void debugControllers(Telemetry telemetry, ControllerInputSystem c1, ControllerInputSystem c2){
        telemetry.addData("Controller Inputs/Controller 1/Buttons", c1.getPressedFaceButtons());
        telemetry.addData("Controller Inputs/Controller 1/Side Buttons/Bumpers", c1.gamepad.left_bumper + " " + c1.gamepad.right_bumper);
        telemetry.addData("Controller Inputs/Controller 1/Side Buttons/Triggers", c1.gamepad.left_trigger + " " + c1.gamepad.right_trigger);
        telemetry.addData("Controller Inputs/Controller 1/Joysticks/Left", "(" + c1.gamepad.left_stick_x + ", " + c1.gamepad.left_stick_y + ")");
        telemetry.addData("Controller Inputs/Controller 1/Joysticks/Right", "(" + c1.gamepad.right_stick_x + ", " + c1.gamepad.right_stick_y + ")");

        telemetry.addData("Controller Inputs/Controller 2/Buttons", c2.getPressedFaceButtons());
        telemetry.addData("Controller Inputs/Controller 2/Side Buttons/Bumpers", c2.gamepad.left_bumper + " " + c2.gamepad.right_bumper);
        telemetry.addData("Controller Inputs/Controller 2/Side Buttons/Triggers", c2.gamepad.left_trigger + " " + c2.gamepad.right_trigger);
        telemetry.addData("Controller Inputs/Controller 2/Joysticks/Left", "(" + c2.gamepad.left_stick_x + ", " + c2.gamepad.left_stick_y + ")");
        telemetry.addData("Controller Inputs/Controller 2/Joysticks/Right", "(" + c2.gamepad.right_stick_x + ", " + c2.gamepad.right_stick_y + ")");
    }

    //DeltaTime Manager
    public static double oldUnixTimestamp = System.nanoTime() * 1e-9;
    public static double currentUnixTimestamp = System.nanoTime() * 1e-9;
    public static double deltaTime = System.nanoTime() * 1e-9;

    public static void updateDeltaTime(){
        oldUnixTimestamp = currentUnixTimestamp;
        currentUnixTimestamp = System.nanoTime() * 1e-9;

        deltaTime = currentUnixTimestamp - oldUnixTimestamp;
    }
}
