package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name = "Main TeleOp")

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
            //driveMotorPositions[i] = new DeltaFloat();
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

    //Scales all drive motor powers until they all fall within the range [-1, 1]
    public void normalizeValues(){
        
    }

    /*Adjusts motor power to account for micro errors with motors
    public void adjustMotorPower(){
        //Uses drive motor encoders to get a delta position vector

        //Updates deltapos to new encoder positions
        for(int i = 0; i < 4; i++){
            driveMotorPostions[i].updatePos(driveMotor[i].getCurrentPosition());
        }

        //Calculates position vectors and combines to get actual heading
        Vector2 motorVectors[] = new Vector2[4];

        //Sets motordirection vectors to direction vector of each mecanum wheel
        motorVectors[0] = new Vector2(driveMotorPositions[0], driveMotorPositions[0]);
        motorVectors[1] = new Vector2(-driveMotorPositions[1], driveMotorPositions[1]);
        motorVectors[2] = new Vector2(-driveMotorPositions[2], driveMotorPositions[2]);
        motorVectors[3] = new Vector2(driveMotorPositions[3], driveMotorPositions[3]);

        //Combines left vectors and right vectors
        Vector2 leftVector = Vector2.combine(motorVectors[0], motorVectors[2]);
        Vector2 rightVector = Vector2.combine(motorVectors[1], motorVectors[3]);

        //Calculates heading
        Vector2 headingVector = Vector2.combine(leftVector, rightVector);

        //Gets expected heading based on controller input
        Vector2 expectedVector = new Vector2(leftStick.x, leftStick.y);

        //Calculates difference between heading and expected heading
        double headingErrorTheta = expectedVector.getTheta() - headingVector.getTheta();

        //Subtracts error angle from expected to get adjusted vector
        Vector2 newVector = new Vector2();
        newVector.x = Math.cos(headingVector.getTheta() - headingErrorTheta);
        newVector.y = Math.sin(headingVector.getTheta() - headingErrorTheta);


    }*/

    //EDIT: DEPRICATED
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
