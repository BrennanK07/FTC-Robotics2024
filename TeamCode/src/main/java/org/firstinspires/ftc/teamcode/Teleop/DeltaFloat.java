package org.firstinspires.ftc.teamcode.Teleop;

/*
    This class helps when dealing with odometry / encoders
*/

public class DeltaFloat{
    double oldPos;
    double currentPos;
    double deltaPos;

    void DeltaFloat(double initVal){
        oldPos = initVal;
        currentPos = initVal;
    }

    void updatePos(double newPos){
        oldPos = currentPos;
        currentPos = newPos;

        deltaPos = currentPos - oldPos;
    }
}