package org.firstinspires.ftc.teamcode.tools;

public class PIDTuner{
    double kP = 1.0;
    double kD = 0.005;
    double kI = 0.05;

    double prevPos;

    double proprtional;
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
        proprtional = targetPos - currentPos; // current error
        integral += currentPos - prevPos; // accumulation of error over domain
        derivative = (currentPos - prevPos) / deltaTime; // dp/dt

        prevPos = currentPos;

        calculatePID();
    }

    public void calculatePID(){
        power = (kP * proprtional) + (kD * derivative) + (kI * integral);
    }
}