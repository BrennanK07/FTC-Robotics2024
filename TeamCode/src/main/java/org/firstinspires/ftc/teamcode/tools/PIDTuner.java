package org.firstinspires.ftc.teamcode.tools;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDTuner{
    double kP = 0.05; //0.1
    double kD = 0.0001; //0.0005
    double kI = 0.01; //0.01

    double prevPos;

    double proportional;
    double derivative;
    double integral;

    public double power;

    public PIDTuner(double currentPos){
        prevPos = currentPos;
    }

    public PIDTuner(double currentPos, double kP, double kD, double kI){
        prevPos = currentPos;
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
    }

    public void updatePID(double currentPos, double targetPos, double deltaTime){
        proportional = targetPos - currentPos; // current error
        integral += currentPos - prevPos; // accumulation of error over domain
        derivative = (currentPos - prevPos) / deltaTime; // dp/dt

        prevPos = currentPos;

        calculatePID();
    }

    public void calculatePID(){
        power = (kP * proportional) + (kD * derivative) + (kI * integral);
    }

    public void debugPID(Telemetry telemetry, String name){
        telemetry.addData(name + "/kP kD kI", kP + " " + kD + " " + kI);

        telemetry.addData(name + "/Proportion e(t)", proportional);
        telemetry.addData(name + "/Integral E(t)", integral);
        telemetry.addData(name + "/Derivative e'(t)", derivative);

        telemetry.addData(name + "/PID Sum", power);
    }
}