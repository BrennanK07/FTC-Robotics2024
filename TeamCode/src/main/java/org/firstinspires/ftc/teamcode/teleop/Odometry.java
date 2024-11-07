package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry extends Main{
    public OdometryArray odometry = new OdometryArray();

    static double LATERAL_DISTANCE = 1.0; //Distance between two lateral wheels
    static double FORWARD_OFFSET = 1.0; //Distance from center of rotation to y-axis wheel
    static double WHEEL_DIAMETER = 1.0; //Diameter of the wheel in mm
    static int WHEEL_TICK_RATE = 2000; //Ticks per revolution of the odometry wheels

    public void main(String args[]){
        odometry.left = hardwareMap.get(DcMotor.class, "front left");
        odometry.right = hardwareMap.get(DcMotor.class, "name");
        odometry.back = hardwareMap.get(DcMotor.class, "name");
    }
    
    class OdometryArray{
        DcMotor left;
        DcMotor right;
        DcMotor back;
        
        DeltaFloat leftOdo = new DeltaFloat();
        DeltaFloat rightOdo = new DeltaFloat();
        DeltaFloat frontOdo = new DeltaFloat();

        //Polls positions and swaps position buffers
        void pollPositions(){
            leftOdo.updatePos(this.left.getCurrentPosition());
            rightOdo.updatePos(this.right.getCurrentPosition());
            frontOdo.updatePos(this.back.getCurrentPosition());

            telemetry.addData("OdoLeft:", leftOdo.currentPos);
            telemetry.addData("OdoRight:", rightOdo.currentPos);
            telemetry.addData("OdoBack:", frontOdo.currentPos);
        }


    }
}
