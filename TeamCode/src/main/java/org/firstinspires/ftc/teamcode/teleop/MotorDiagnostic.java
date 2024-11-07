package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive Motor Diagnostic")

public class MotorDiagnostic extends LinearOpMode{
    private final DcMotor[] driveMotor = new DcMotor[4]; //[fl, fr, bl, br]
    private DcMotor wormGear;
    private final double[] driveMotorPower = new double[4]; //Untransformed motor power
    
    private double MOTOR_SPEED = 0.0;

    private int activeTestMotor = 0; //Follows order of driveMotor array
    private final String[] motorNames = {"front left", "front right", "back left", "back right"};

    private boolean isHalted = true;
    
    private final boolean[] previousInput = new boolean[4]; //up, down, a, b
    
    @Override
    public void runOpMode(){
        initMotors();

        waitForStart();

        while(opModeIsActive()){
            //Adjust motor speed with d-pad
            if(gamepad1.dpad_up && !previousInput[0]){
                MOTOR_SPEED += 0.1;
                previousInput[0] = true;
            }else{
                previousInput[0] = false;
            }
            
            if(gamepad1.dpad_down && !previousInput[1]){
                MOTOR_SPEED -= 0.1;
                
                previousInput[1] = true;
            }else{
                previousInput[1] = false;
            }

            if(MOTOR_SPEED > 1f){
                MOTOR_SPEED = 1;
            }
            else if(MOTOR_SPEED < -1f){
                MOTOR_SPEED = -1;
            }

            //Changes current test motor
            if(gamepad1.a && !previousInput[2]){
                activeTestMotor += 1;

                if(activeTestMotor > 3){
                    activeTestMotor = 0;
                }
                
                previousInput[2] = true;
            }else{
                previousInput[2] = false;
            }

            //Worm Gear Check
            if(gamepad1.right_trigger != 0f){
                wormGear.setPower(1.0f);
            }else{
                wormGear.setPower(0.0f);
            }

            //Temp halt button
            if(gamepad1.b && !previousInput[3]){
                isHalted = !isHalted;

                //Stops b button press events activating on every frame
                previousInput[3] = true;
            }else{
                previousInput[3] = false;
            }
            
            for(int i = 0; i < 4; i++){
                driveMotorPower[i] = 0;
            }
            
            driveMotorPower[activeTestMotor] = MOTOR_SPEED;

            setDriveMotors(driveMotorPower);

            if(isHalted){
                telemetry.addData("Motor Speed: ", "HALTED");
                MOTOR_SPEED = 0;
            }
            else{
                telemetry.addData("Motor Speed: ", MOTOR_SPEED);
            }
            
            telemetry.addData("Motor testing: ", motorNames[activeTestMotor]);
            telemetry.update();
        }
    }

    public void initMotors(){
        driveMotor[0] = hardwareMap.get(DcMotor.class, "front left");
        driveMotor[1] = hardwareMap.get(DcMotor.class, "front right");
        driveMotor[2] = hardwareMap.get(DcMotor.class, "back left");
        driveMotor[3] = hardwareMap.get(DcMotor.class, "back right");

        //Worm gear
        wormGear = hardwareMap.get(DcMotor.class, "worm gear");
        
        telemetry.addData("Status", "Initalized");
        telemetry.update();
    }

    public void setDriveMotors(double[] motorPowers){
        for(int i = 0; i < 4; i++){
            if(i == 1 || i == 3){ //Fixes motor spinning backwards
                motorPowers[i] = -motorPowers[i];
            }
            
            driveMotor[i].setPower(motorPowers[i]);
        }
    }
}