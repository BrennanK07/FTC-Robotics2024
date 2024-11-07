package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Main TeleOp")

public class Main extends LinearOpMode{
    //Motors
    private final DcMotor[] driveMotor = new DcMotor[4]; //[fl, fr, bl, br]
    private final double[] driveMotorPower = new double[4]; //Untransformed motor power
    private DcMotor linearSlideMotor;
    private CRServo intakeMotor;
    private DcMotor linearSlidePosMotor;
    
    static double MOTOR_SPEED = 0.9; //Default 0.9
    static double ROTATION_SPEED = 1.0; //Default 0.75

    //Inputs
    private final Vector2 leftStick = new Vector2();
    private final Vector2 rightStick = new Vector2();
    static double STICK_DEADZONE = 0.1;

    private double slideAxis = 0.0;
    private double slidePosAxis = 0.0;
    private double intakeAxis = 0.0;

    //Constants
    private final DeltaFloat[] driveMotorPositions = new DeltaFloat[4];
    static double MECANUM_TICK_RATE = 537.7; //DriveMotorPos / MECANUM_TICK_RATE = totalRotations
    //static double DRIVE_MOTOR_MAX_RPM = 312;

    static double MAX_SLIDE_EXTENSION = 537.7 * 4.1; //4.1 rotations
    static double MIN_SLIDE_EXTENSION = 0.0;

    static boolean SHOW_DEBUG_VALUES = true;
    
    double oldUnixTimestamp = System.nanoTime() * 1e-9;
    double unixTimestamp = System.nanoTime() * 1e-9;
    double deltaTime;

    double currentServoPos = 0;
    static double SERVO_SPEED = 1;

    //Drive Adjustment
    public Vector2 driveDirection = new Vector2();
    public Vector2 rotationHeading = new Vector2();
    
    @Override
    public void runOpMode(){
        //Initializes motors
        driveMotor[0] = hardwareMap.get(DcMotor.class, "front left");
        driveMotor[1] = hardwareMap.get(DcMotor.class, "front right");
        driveMotor[2] = hardwareMap.get(DcMotor.class, "back left");
        driveMotor[3] = hardwareMap.get(DcMotor.class, "back right");

        linearSlideMotor = hardwareMap.get(DcMotor.class, "worm_gear");
        intakeMotor = hardwareMap.get(CRServo.class, "intake");
        linearSlidePosMotor = hardwareMap.get(DcMotor.class, "slide");

        //Setup encoder types
        linearSlidePosMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlidePosMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Init drive motor encoders
        for(int i = 0; i < 4; i++){
            driveMotorPositions[i] = new DeltaFloat();
            driveMotorPositions[i].updatePos(driveMotor[i].getCurrentPosition() / MECANUM_TICK_RATE);
        }
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        //Wait for driver to press play
        waitForStart();
        
        while(opModeIsActive()){
            updateControllerInputs();
            updateDeltaTime();
            
            //adjustMotorPower();
            setDriveMotors(driveMotorPower);
            
            telemetry.addData("Status", "Running");
            
            if(SHOW_DEBUG_VALUES){
                telemetry.addData("Left Stick Position:", leftStick.x + ", " + leftStick.y);
                telemetry.addData("Right Stick Position:", rightStick.x + ", " + rightStick.y);
                //telemetry.addData("Motor Position TEST", driveMotor[0].getCurrentPosition());

                /*
                for(int i = 0; i < 4; i++){
                    telemetry.addData("Wheel Velocity Test " + i + " ", (driveMotorPositions[i].deltaPos) / (deltaTime * MECANUM_TICK_RATE));
                }
                */

                telemetry.addData("deltaTime (ms) ", (int)(deltaTime * 1000));
                telemetry.addData("intake Axis", intakeAxis);
                telemetry.addData("Linear Slide Encoder Position", linearSlidePosMotor.getCurrentPosition());
            }
            
            telemetry.update();
        }
    }

    public void updateDeltaTime(){
        oldUnixTimestamp = unixTimestamp;
        unixTimestamp = System.nanoTime() * 1e-9;
        deltaTime = unixTimestamp - oldUnixTimestamp;
    }

    public void updateControllerInputs(){
        //Controller input
        leftStick.x = gamepad1.left_stick_x;
        leftStick.y = gamepad1.left_stick_y;
            
        //Dpad inputs
        if(gamepad1.dpad_up){
            leftStick.y = -1;
        }
        if(gamepad1.dpad_down){
            leftStick.y = 1;
        }
        if(gamepad1.dpad_left){
            leftStick.x = -1;
        }
        if(gamepad1.dpad_right){
            leftStick.x = 1;
        }
            
        //Updates drive motor power and formats values
        leftStick.y = fixValue(leftStick.y) * MOTOR_SPEED;
        leftStick.x = fixValue(leftStick.x) * MOTOR_SPEED;
            
        driveMotorPower[0] = leftStick.y - leftStick.x;
        driveMotorPower[1] = leftStick.y + leftStick.x;
        driveMotorPower[2] = leftStick.y + leftStick.x;
        driveMotorPower[3] = leftStick.y - leftStick.x;

        //Rotation
        //Adds stick val to driveMotorPower (on top of crab walk speed)
        //"Normalizes" the values to make them all less than or equal to 1
        rightStick.x = gamepad1.right_stick_x;
        rightStick.y = gamepad1.right_stick_y;
            
        rightStick.y = fixValue(rightStick.y);
        rightStick.x = fixValue(rightStick.x);
            
        if(rightStick.x != 0){
            driveMotorPower[0] += -rightStick.x * ROTATION_SPEED;
            driveMotorPower[2] += -rightStick.x * ROTATION_SPEED;
            driveMotorPower[1] += rightStick.x * ROTATION_SPEED;
            driveMotorPower[3] += rightStick.x * ROTATION_SPEED;
        }

        //Combined Directional Vector

        //Intake / slide
        slideAxis = fixValue(gamepad2.left_stick_y);
        slidePosAxis = fixValue(gamepad2.right_stick_y);

        //LT for neg, RT for pos
        if(gamepad2.right_trigger > 0){
            intakeAxis = fixValue(gamepad2.right_trigger);
        }
        else if(gamepad2.left_trigger > 0){
            intakeAxis = -fixValue(gamepad2.left_trigger);
        }else{
            intakeAxis = 0;
        }
    }
    
    public boolean isExceedingMaxPower(double[] magnitudes){
        for (double magnitude : magnitudes) {
            if (Math.abs(magnitude) > 1) {
                return true;
            }
        }

        return false;
    }
    
    //Sets the values of the drive motors from the driveMotorPower[] array
    public void setDriveMotors(double[] motorPowers){
        //Drive motors
        for(int i = 0; i < 4; i++){
            if(i == 1 || i == 3){ //Fixes motor spinning backwards
                motorPowers[i] = -motorPowers[i];
            }
            
            driveMotor[i].setPower(motorPowers[i]);
            //driveMotor[i].setPower(lerp(driveMotor[i].getPower(), motorPowers[i], ACCEL_RATE));
        }

        //Linear slide
        linearSlideMotor.setPower(slideAxis);

        //Intake Servo
        //double currentServoPos = intakeMotor.getPosition();

        currentServoPos += intakeAxis * SERVO_SPEED * deltaTime;

        intakeMotor.setPower(currentServoPos + (intakeAxis * SERVO_SPEED * deltaTime));
        //intakeMotor.setPower(intakeAxis);

        //Linear slide
        linearSlidePosMotor.setTargetPosition((int)(slidePosAxis * MAX_SLIDE_EXTENSION));

        //Stops linear slide from exceeding its bounds
        if(linearSlidePosMotor.getCurrentPosition() >= (int)MAX_SLIDE_EXTENSION){
            linearSlidePosMotor.setTargetPosition((int)MAX_SLIDE_EXTENSION);
        }
        if(linearSlidePosMotor.getCurrentPosition() <= (int)MIN_SLIDE_EXTENSION){
            linearSlidePosMotor.setTargetPosition((int)MIN_SLIDE_EXTENSION);
        }
    }
    
    //Helps with analog stick accuracy by forcing an input greater than +THRESHOLD or -THRESHOLD to register
    public double fixValue(double value){
        if(Math.abs(value) < STICK_DEADZONE){
            return 0;
        }else{
            return value;
        }
    }

    //Adjusts motor power to account for micro errors with motors
    //EDIT: Deprecated and replaced by roadrunner
    /*
    public void adjustMotorPower(){
        //Uses drive motor encoders to get a delta position vector

        //"driveMotorPositions" calculated in 1.0 / revolution
        for(int i = 0; i < 4; i++){
            driveMotorPositions[i].updatePos(driveMotor[i].getCurrentPosition() / MECANUM_TICK_RATE);
        }

        //Defines vectors for all drive motors
        double[] motorScalar = new double[4];

        for(int i = 0; i < 4; i++){
            motorScalar[i] = driveMotorPositions[i].deltaPos / (deltaTime * MECANUM_TICK_RATE);
        }

        //Defines expected output vectors for drive motors, which are changed by outside factors
        //expectedVector is the same as driveMotorPower but expectedVector sounds cooler
        double[] expectedScalar = new double[4];

        for(int i = 0; i < 4; i++){
            expectedScalar[i] = driveMotorPower[i];
        }

        double[] generatedScalar = new double[4];

        //Calculations to reposition vector to account for drift and strafe
        //All angles calculated in radians

        //Left hand side

        //Combine left and right vectors and divide y/x and find arctan
        double actualTheta = Math.atan2((motorScalar[0] + motorScalar[2]), (motorScalar[0] - motorScalar[2]));
        double expectedTheta = Math.atan2(expectedScalar[0] + expectedScalar[2], expectedScalar[0] - expectedScalar[2]);

        telemetry.addData("actualAngle ", actualTheta);
        telemetry.addData("expectedAngle ", expectedTheta);

        //New angle is left offset instead of right offset
        double adjustedTheta = expectedTheta + actualTheta - (Math.PI / 4);

        //Convert back to vectors and wheel powers
        //When rotated -pi/4, mecanum wheel vectors match x and y directions
        Vector2 adjustedVector = new Vector2(Math.cos(adjustedTheta), Math.sin(adjustedTheta));

        generatedScalar[0] = adjustedTheta.x;
        generatedScalar[1] = adjustedTheta.y;
        generatedScalar[2] = adjustedTheta.y;
        generatedScalar[3] = adjustedTheta.x;

        //Temporary set drive motor power to equal generated vectors
        for(int i = 0; i < 4; i++)[
            driveMotorPower[i] = generatedScalar[i];
            telemetry.addData("Motor " + i + " adjusted speed", generatedScalar[i]);
        ]
    }*/

    //EDIT: DEPRECIATED
    /*
    public double lerp(double start, double end, double speed){
        //Helps with acceleration
        //transitions between first and second value
        //Returns start

        if(Math.abs(start - end) < 0.01){
            return 0;
        }

        return (start * (1.0 - speed)) + (end * speed);
    }*/
}
