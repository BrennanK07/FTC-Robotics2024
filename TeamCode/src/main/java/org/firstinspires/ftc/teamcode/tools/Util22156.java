package org.firstinspires.ftc.teamcode.tools;

/*
 * INSTRUCTIONS FOR USE (you're welcome chris):
 * DO NOT INSTANTIATE THIS CLASS AS AN OBJECT
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

    public static double clamp(double val, int min, int max){
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

    //Vector Math and Conversions
    // import org.firstinspires.ftc.teamcode.tools.Util22156.Vector2;
    public static class Vector2{
        double x;
        double y;

        public Vector2(double x, double y){
            this.x = x;
            this.y = y;
        }

        public Vector2(){
            this(0, 0);
        }

        public Vector2 add(Vector2 a, Vector2 b){
            return new Vector2(a.x + b.x, a.y + b.y);
        }

        public Vector2 subtract(Vector2 a, Vector2 b){
            return new Vector2(a.x - b.x, a.y - b.y);
        }

        public double dotProduct(Vector2 a, Vector2 b){
            return (a.x * b.x) + (a.y * b.y);
        }

        public Vector2 normalize(Vector2 a){
            double magnitude = Math.sqrt(Math.pow(a.x, 2) + Math.pow(a.y, 2));
            return new Vector2(a.x / magnitude, a.y / magnitude);
        }

        public double getAngle(Vector2 a){
            return Math.atan2(a.x, a.y);
        }
    }

    public double rad(double theta){
        return theta * (Math.PI / 180);
    }

    public double deg(double theta){
        return theta * (180 / Math.PI);
    }
}
