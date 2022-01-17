package org.firstinspires.ftc.teamcode;
//done other than testing?
import static java.lang.Math.abs;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

public class AutoMethods{
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     servoTimer = new ElapsedTime();
    public ElapsedTime     carouselTimer = new ElapsedTime();

    final int     COUNTS_PER_MOTOR_REV          = 560 ;    // eg: TETRIX Motor Encoder
    final double  DRIVE_GEAR_REDUCTION          = 18.8803;     // This is < 1.0 if geared UP
    final double     WHEEL_SIRCONFERENCE_INCHES    = 11.78097;     // For figuring circumference
    final double     COUNTS_PER_INCH             = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_SIRCONFERENCE_INCHES);
    final double     DRIVE_SPEED                 = 0.6;
    final double     TURN_SPEED                  = 0.5;
    final double     DISTANCE_PER_DEG_TURNED = 0.19987416;


    
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;
    public DcMotorEx carouselMotor;
    public Servo holderServo;
    public DcMotorEx elevatorMotor;
    public DcMotorEx intakeMotor;
    
    

    // P functionpublic static final double kP = 0.1 // placeholder value that needs to be tested

    // double errorSum = 0.0;
    // double error = 0.0;
    // double mecanumPower = 0.0;
    // double previousError = 0.0;
    // double maxPotentialOvershoot = .01;


    double destinationFeet;
    double currentPositionFeet;

    public AutoMethods (DcMotorEx leftFrontMotor, DcMotorEx rightFrontMotor, DcMotorEx leftBackMotor, DcMotorEx rightBackMotor, DcMotorEx carouselMotor, Servo holderServo, DcMotorEx elevatorMotor, DcMotorEx intakeMotor){
        
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.carouselMotor = carouselMotor;
        this.holderServo = holderServo;
        this.elevatorMotor = elevatorMotor;
        this.intakeMotor = intakeMotor;
        
    }

    //PID methods
    public void setAllPIDFCoefs(double p, double i, double d, double f){
        leftFrontMotor.setVelocityPIDFCoefficients(p,i,d,f);
        rightFrontMotor.setVelocityPIDFCoefficients(p,i,d,f);
        leftBackMotor.setVelocityPIDFCoefficients(p,i,d,f);
        rightBackMotor.setVelocityPIDFCoefficients(p,i,d,f);
    }
    public void setAllPositionPIDFCoefs(double p){
        leftFrontMotor.setPositionPIDFCoefficients(p);
        rightFrontMotor.setPositionPIDFCoefficients(p);
        leftBackMotor.setPositionPIDFCoefficients(p);
        rightBackMotor.setPositionPIDFCoefficients(p);
    }
    

    //encoder methods
    public void resetAllEncoders(){
        leftFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){
        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
    public void runUsingPositions(){
        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
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
    }
    public void turnOnMotors() {
        rightFrontMotor.setMotorEnable();
        leftFrontMotor.setMotorEnable();
        rightBackMotor.setMotorEnable();
        leftBackMotor.setMotorEnable();
    }
    public void turnOffMotors() {
        rightFrontMotor.setMotorDisable();
        leftFrontMotor.setMotorDisable();
        rightBackMotor.setMotorDisable();
        leftBackMotor.setMotorDisable();
    }
    
    
    //

    //target setting methods
    //TODO: Mabye put /2 or *2 or smth for mecanum targets since they might travel a different amount of distance on each rotation
    public void setFBTargets(double inches){
        rightFrontMotor.setTargetPosition((int)(inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        leftFrontMotor.setTargetPosition((int)(inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        rightBackMotor.setTargetPosition((int)(inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        leftBackMotor.setTargetPosition((int)(inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
    }
    public void setMecTargets(double inches){
        rightFrontMotor.setTargetPosition(-(int)((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftFrontMotor.setTargetPosition((int)((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        rightBackMotor.setTargetPosition((int)((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftBackMotor.setTargetPosition(-(int)((inches/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
    }
    public void setTurnTargets(double degrees){
        rightFrontMotor.setTargetPosition(-(int)(((degrees*DISTANCE_PER_DEG_TURNED)/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftFrontMotor.setTargetPosition((int)(((degrees*DISTANCE_PER_DEG_TURNED)/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        rightBackMotor.setTargetPosition(-(int)(((degrees*DISTANCE_PER_DEG_TURNED)/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftBackMotor.setTargetPosition((int)(((degrees*DISTANCE_PER_DEG_TURNED)/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
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

    public void driveToPosition(double sideways, double straight, double speed){
        setMecTargets(sideways);
        turnOnMotors();  //sideways movement
        setAllMecPows(speed);
        resetEncodersAfterMovementComplete();
        turnOffMotors();
        setAllMecPows(speed);
        setFBTargets(straight);
        turnOnMotors();  // straight movement
        resetEncodersAfterMovementComplete();
        turnOffMotors();
    }

    //servo methods
    public void setServoPos(double pos){
        holderServo.setPosition(pos);
    }
    public void setServoPlace(int place){
        if (place == 1){
            setServoPos(0.3); //placeholder
        }
        else if (place == 2){
            setServoPos(0.5); //placeholder
        }
        else {
            setServoPos(0); //placeholder
        }
    }

    //elevator methods
    public void setElevatorPos(int pos){
        if (pos == 1){
            elevatorMotor.setTargetPosition(1000); //placeholder
        }
        else if (pos == 2){
            elevatorMotor.setTargetPosition(2000); //placeholder
        }
        else {
            elevatorMotor.setTargetPosition(0); //placeholder
        }
        elevatorMotor.setVelocity(.4);
        while(elevatorMotor.getVelocity() > .05){
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //intake methods
    public void intakeOn(double power){
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



