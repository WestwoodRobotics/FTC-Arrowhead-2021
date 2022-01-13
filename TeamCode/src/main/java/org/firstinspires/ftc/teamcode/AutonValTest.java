package org.firstinspires.ftc.robotcontroller.external.samples;
//done other than testing?
import static java.lang.Math.abs;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    double mecanumPower;
    double destinationFeet;
    double currentPositionFeet;

    //PID methods
    public void setAllPIDFCoefs(double p, double i, double d, double f){

        elevatorMotor.setVelocityPIDFCoefficients(p,i,d,f);
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