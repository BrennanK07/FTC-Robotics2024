package org.firstinspires.ftc.teamcode.teleop;

public class Vector2{
    double x;
    double y;

    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector2(){
        this.x = 0;
        this.y = 0;
    }

    public Vector2 combine(Vector2 a, Vector2 b){
        Vector2 combined = new Vector2(0, 0);

        combined.x = a.x + b.x;
        combined.y = a.y + b.y;

        return combined;
    }
}
