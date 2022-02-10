package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="Cool Auton")


public class OGSureYeah extends LinearOpMode {


    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     servoTimer = new ElapsedTime();
    private ElapsedTime     PIDTime = new ElapsedTime();
    private ElapsedTime     intakeTime = new ElapsedTime();


    static final double     COUNTS_PER_MOTOR_REV          = 28;
    static final double     DRIVE_GEAR_REDUCTION          =  18.9;
    static final double     Motor_SIRCONFERENCE_INCHES    = 11.8677165 ;
    static final double     COUNTS_PER_INCH             = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /Motor_SIRCONFERENCE_INCHES;
//     static final double     DRIVE_speade                 = 0.6;
//     static final double     TURN_speade                  = 0.5;
//     static final double     ELEVATOR_GEAR_RATIO         = 50.9;
//     static final double     COUNTS_PER_ELEVATOR_REV     = 1425;
//     static final double     MAX_ELEVATOR_CAPABLITY      = 123;
//     static final double     ELEVATOR_SPOOL_DIAMETER_INCHES = 1.49606;
//     static final double     ELE_TICKS_PER_INCH   = (ELEVATOR_GEAR_RATIO*COUNTS_PER_ELEVATOR_REV)/ELEVATOR_SPOOL_DIAMETER_INCHES; //someone check my math pls
//     static final int     TICKS_PER_REVOLUTION          = DRIVE_GEAR_REDUCTION*COUNTS_PER_MOTOR_REV;


    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx carouselMotor;
    private CRServo holderServo;
    private DcMotorEx elevatorMotor;
    private DcMotorEx intakeMotor;



//     // P function
//     final double kP = 0.1;// placeholder value that needs to be tested
//     final double kI = 1.0;
//     final double kD = 0.1;
//     double errorSum = 0.0;
//     double error = 0.0;
//     double previousError = 0.0;
//     double maxPotentialOvershoot = .01;
//     final double distancePerDegTurned = 0.01665618;
//     double mecanumPower = 0.8;
//     double destinationFeet;
//     double currentPositionFeet;
//     double orientation = 0;

    public void setFrontBack(int inches) {
        leftFrontMotor.setTargetPosition(inches*(int)COUNTS_PER_INCH);
        rightFrontMotor.setTargetPosition(inches*(int)COUNTS_PER_INCH);
        leftBackMotor.setTargetPosition(inches*(int)COUNTS_PER_INCH);
        rightBackMotor.setTargetPosition(inches*(int)COUNTS_PER_INCH);
    }
    public void setLeftRight(int inches){
        leftFrontMotor.setTargetPosition(inches*(int)COUNTS_PER_INCH);
        rightFrontMotor.setTargetPosition(-inches*(int)COUNTS_PER_INCH);
        leftBackMotor.setTargetPosition(-inches*(int)COUNTS_PER_INCH);
        rightBackMotor.setTargetPosition(inches*(int)COUNTS_PER_INCH);
    }
    public void setPowers(double power){
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
    }
    public void setFrontBackVelocity(int inches,double mod) {
        leftFrontMotor.setVelocity(inches*(int)(mod*COUNTS_PER_INCH));
        rightFrontMotor.setVelocity(inches*(int)(mod*COUNTS_PER_INCH));
        leftBackMotor.setVelocity(inches*(int)(mod*COUNTS_PER_INCH));
        rightBackMotor.setVelocity(inches*(int)(mod*COUNTS_PER_INCH));
    }
    public void setLeftRightVelocity(int inches,double mod) {
        leftFrontMotor.setVelocity(inches*(int)(mod*COUNTS_PER_INCH));
        rightFrontMotor.setVelocity(-inches*(int)(mod*COUNTS_PER_INCH));
        leftBackMotor.setVelocity(-inches*(int)(mod*COUNTS_PER_INCH));
        rightBackMotor.setVelocity(inches*(int)(mod*COUNTS_PER_INCH));
    }
    public void setPositionPID(int P){
        leftFrontMotor.setPositionPIDFCoefficients(P);
        rightFrontMotor.setPositionPIDFCoefficients(P);
        leftBackMotor.setPositionPIDFCoefficients(P);
        rightBackMotor.setPositionPIDFCoefficients(P);
    }
    public void setVelocityPID(double p, double i, double d, double f){
        leftFrontMotor.setVelocityPIDFCoefficients(p,i,d,f);
        rightFrontMotor.setVelocityPIDFCoefficients(p,i,d,f);
        leftBackMotor.setVelocityPIDFCoefficients(p,i,d,f);
        rightBackMotor.setVelocityPIDFCoefficients(p,i,d,f);
    }
    public void stopMotors(){
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

       
    public void runOpMode() {
        leftFrontMotor  = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBackMotor");
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        holderServo = hardwareMap.get(CRServo.class, "holderServo");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
        holderServo.setDirection(CRServo.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorEx.Direction.FORWARD);

        
        telemetry.addData("Status", "Initialized");




        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Parth0",  "Starting at %7d :%7d");
        telemetry.update();
        setVelocityPID(10,0,3,13.703116805);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Go somewhere
        //setFrontBackVelocity(100);
        // setPowers(1);
        // leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while(runtime.milliseconds() < 400){
            setLeftRightVelocity((int)-(COUNTS_PER_INCH),1.4);
        }
        stopMotors();
        runtime.reset();
        setFrontBackVelocity((int)-(COUNTS_PER_INCH),1);
        sleep(300);
        // while(runtime.milliseconds() < 300){
        //     setFrontBackVelocity((int)-(COUNTS_PER_INCH),1);
        // }
        stopMotors();
        runtime.reset();
        carouselMotor.setPower(0.15);
        sleep(6000);
        // while(runtime.milliseconds() < 6000){
        //     carouselMotor.setPower(0.15);
        // }
        stopMotors();
        carouselMotor.setPower(0);
        runtime.reset();
        while(runtime.milliseconds() < 600){
            setLeftRightVelocity((int)-(COUNTS_PER_INCH),1.4);
        }
        stopMotors();
        runtime.reset();
        while(runtime.milliseconds() < 3500){
            setFrontBackVelocity((int)(COUNTS_PER_INCH),1.2);
            
            
        }
        
        
        
        
       
    }

}
