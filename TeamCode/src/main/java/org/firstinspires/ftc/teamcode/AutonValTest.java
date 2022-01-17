package org.firstinspires.ftc.robotcontroller.external.samples;
//done other than testing?
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
public class AutonValTest {
    private ElapsedTime     runtime = new ElapsedTime();


    private Servo holderServo;
    private DcMotorEx elevatorMotor;
    private DcMotorEx intakeMotor;



    // P functionpublic static final double kP = 0.1 // placeholder value that needs to be tested

    private ElapsedTime     servoTimer = new ElapsedTime();
    private ElapsedTime     PIDTime = new ElapsedTime(ElapsedTime.resolution milliseconds);
    public interface Telemetry
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
    public void setAllPIDFCoefs(double p, double i, double d, double f){

        elevatorMotor.setVelocityPIDFCoefficients(p,i,d,f);
        leftFrontMotor.setVelocityPIDFCoefficients(p,i,d,f);
        rightFrontMotor.setVelocityPIDFCoefficients(p,i,d,f);
        leftBackMotor.setVelocityPIDFCoefficients(p,i,d,f);
        rightBackMotor.setVelocityPIDFCoefficients(p,i,d,f);
    }

    //encoder methods
    public void resetAllEncoders(){

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runUsingEncoders(){

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    double elevatorPosTicks = 72537.59;
    public int motorLocationTelems(){
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public motorLocationTelems(){
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorPosTicks = 0;

        while (runtime.milliseconds() < 1000000){
            telemetry.addData("elapsedTime", "%.3f", elapsedSeconds);
            elevatorPosTicks = elevatorMotor.getCurrentPosition();
            telemetry.addData("elevatorMotorTicks", "%.3f", elevatorPosTicks);
            telemetry.addData("ServoPos", "%.3f", holderServo.getPosition());
            telemetry.update();
        }
        return 1;
    }

    }
    motorLocationTelems();

}
