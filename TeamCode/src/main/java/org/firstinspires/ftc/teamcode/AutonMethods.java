package org.firstinspires.ftc.robotcontroller.external.samples;
//done other than testing?
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;


//https://github.com/FIRST-Tech-Challenge/skystone/wiki/Using-Computer-Vision-in-FTC
public class AutonMethods extends OGSureYeah {
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     servoTimer = new ElapsedTime();
    public ElapsedTime     carouselTimer = new ElapsedTime();

    final double     COUNTS_PER_MOTOR_REV          = 560 ;    // eg: TETRIX Motor Encoder
    final double     DRIVE_GEAR_REDUCTION          = 20 ;     // This is < 1.0 if geared UP
    final double     WHEEL_SIRCONFERENCE_INCHES    = 11.78097;     // For figuring circumference
    final double     COUNTS_PER_INCH             = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_SIRCONFERENCE_INCHES);
    final double     DRIVE_SPEED                 = 0.6;
    final double     TURN_SPEED                  = 0.5;
    final double     DISTANCE_PER_DEG_TURNED = 0.19987416;
    public static final double kI = 1.0
    public static final double kD = 0.1

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx carouselMotor;
    private Servo holderServo;
    private DcMotorEx elevatorMotor;
    private DcMotorEx intakeMotor;
    leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
    rightFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
    leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
    rightBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
    carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
    holderServo.setDirection(Servo.Direction.FORWARD);
    elevatorMotor.setDirection(DcMotorEx.Direction.FORWARD);
    intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

    // P functionpublic static final double kP = 0.1 // placeholder value that needs to be tested

    double errorSum = 0.0
    double error = 0.0
    double mecanumPower = 0.0;
    double previousError = 0.0;
    double maxPotentialOvershoot = .01;

    double mecanumPower;
    double destinationFeet;
    double currentPositionFeet;

    //PID methods
    public void setAllPIDFCoefs(double p, double i, double d, double f){              // Is setVelocityPIDFCoefficients for the setVelocity commands?
        leftFrontMotor.setVelocityPIDFCoefficients(p,i,d,f);
        rightFrontMotor.setVelocityPIDFCoefficients(p,i,d,f);
        leftBackMotor.setVelocityPIDFCoefficients(p,i,d,f);
        rightBackMotor.setVelocityPIDFCoefficients(p,i,d,f);
    }

    //encoder methods
    public void resetAllEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetEncodersAfterMovementComplete(){
        while (leftFrontMotor.getVelocity() > 0.05 || rightFrontMotor.getVelocity() > 0.05 || leftBackMotor.getVelocity() > 0.05 || rightBackMotor.getVelocity() > 0.05){
            //waits until the motors are done
        }
        //once the motors are done moving, this method resets the encoders
        if (!(leftFrontMotor.getVelocity() > 0.05 || rightFrontMotor.getVelocity() > 0.05 || leftBackMotor.getVelocity() > 0.05 || rightBackMotor.getVelocity() > 0.05)){
            resetAllEncoders();
        }
    }

    //power setting methods
    public void setAllPows(double power){
        rightFrontMotor.setVelocity(power);
        leftFrontMotor.setVelocity(power);
        rightBackMotor.setVelocity(power);
        leftBackMotor.setVelocity(power);
    }
    public void setAllMecPows(double power){  //what does mec mean
        rightFrontMotor.setVelocity(-power);
        leftFrontMotor.setVelocity(power);
        rightBackMotor.setVelocity(power);
        leftBackMotor.setVelocity(-power);
    }
    public void turnPows(double power) {      // what's this vs mec
        rightFrontMotor.setVelocity(-power);
        leftFrontMotor.setVelocity(power);
        rightBackMotor.setVelocity(-power);
        leftBackMotor.setVelocity(power);
    }//

    //target setting methods
    //TODO: Mabye put /2 or *2 or smth for mecanum targets since they might travel a different amount of distance on each rotation
    public void setFBTargets(double inches){
        rightFrontMotor.setTargetPosition((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        leftFrontMotor.setTargetPosition((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        rightBackMotor.setTargetPosition((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        leftBackMotor.setTargetPosition((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
    }
    public void setMecTargets(double inches){
        rightFrontMotor.setTargetPosition(-((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftFrontMotor.setTargetPosition(((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        rightBackMotor.setTargetPosition(((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftBackMotor.setTargetPosition(-((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
    }
    public void setTurnTargets(double degrees){
        rightFrontMotor.setTargetPosition(-(((degrees*DISTANCE_PER_DEG_TURNED)/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftFrontMotor.setTargetPosition((((degrees*DISTANCE_PER_DEG_TURNED)/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        rightBackMotor.setTargetPosition(-(((degrees*DISTANCE_PER_DEG_TURNED)/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftBackMotor.setTargetPosition((((degrees*DISTANCE_PER_DEG_TURNED)/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
    }

    //drive methods in inches (group previous few methods)
    public void driveTo(double sideways, double straight, double speed){
        setMecTargets(sideways);
        setFBTargets(straight);
        setAllMecPows(speed);  //sideways movement
        resetEncodersAfterMovementComplete();
        setAllPows(speed);  // straight movement
        resetEncodersAfterMovementComplete();
    }

    //servo methods
    public void setServoPos(double pos){
        holderServo.setPosition(pos)
    }
    public void setServoPlace(int place){
        if (place == 1){
            setServoPos(/*TODO: measure the position vals*/)
        }
        else if (place == 2){
            setServoPos(/*TODO: measure the position vals*/);
        }
        else {
            setServoPos(/*TODO: measure the position vals*/)
        }
    }

    //elevator methods
    public void setElevatorPos(int pos){
        if (pos == 1){
            elevatorMotor.setTargetPosition(/*TODO: the math*/);
        }
        else if (pos == 2){
            elevatorMotor.setTargetPosition(/*TODO: the math*/);
        }
        else {
            elevatorMotor.setTargetPosition(/*TODO: the math*/);
        }
        elevatorMotor.setVelocity(.4);
        while(elevatorMotor.getVelocity > .05){}
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //intake methods
    public void intakeOn(power){
        intakeMotor.setPower(power);
    }
    public void intakeOff(){
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // carousel methods
    public void carouselOn(double power, int seconds){
        carouselTimer.reset();
        while (carouselTimer.seconds() < seconds){
            carouselMotor.setPower(power);
        }
        carouselMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
    }
    
    /*
    public void carouselOff(){
        carouselMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    */

}
