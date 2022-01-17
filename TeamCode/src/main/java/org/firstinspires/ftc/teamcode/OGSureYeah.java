package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Iterative Opmode")
@Autonomous(name="Good Autonomous", group="Iterative Opmode")

/*
TODO-LIST:
|-------------------------------------------------------|
|*elevator positions                                    |
|*delete pretty much everything on here                 |
|-------------------------------------------------------|
*/

/*public class PushbotAutoDriveByEncoder_Linear extends LinearOpMode {*/
public class OGSureYeah extends LinearOpMode {


    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime PIDTime = new ElapsedTime(ElapsedTime.resolution milliseconds);

    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20;     // This is < 1.0 if geared UP
    static final double WHEEL_SIRCONFERENCE_INCHES = 11.78097;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_SIRCONFERENCE_INCHES);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx carouselMotor;
    private Servo holderServo;
    private DcMotorEx elevatorMotor;
    leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
    rightFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
    leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
    rightBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
    carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
    holderServo.setDirection(Servo.Direction.FORWARD);
    elevatorMotor.setDirection(DcMotorEx.Direction.FORWARD);

    // P function
    public static final double kP = 0.1 // placeholder value that needs to be tested
    public static final double kI = 1.0
    public static final double kD = 0.1
    double errorSum = 0.0
    double error = 0.0
    double mecanumPower = 0.0;
    double previousError = 0.0;
    double maxPotentialOvershoot = .01;
    final double distancePerDegTurned = 0.01665618;
    double mecanumPower;
    double destinationFeet;
    double currentPositionFeet;



}
    

    public void runOpMode() {
        
        // ArrowheadPurelyOriginalWorkDoNotCopyTensorFlowMethods.something something scan file

        AutonMethods.setAllPIDFCoefs(0,0,0,0); 
        AutonMethods.resetAllEncoders();
        AutonMethods.runUsingEncoders();

        Orient the robot with the elevator facing towards the barcode. (The back of the robot?)


        AutonMethods.driveTo( 27 , -15 , 0.5);  // Go to Alliance shipping hub

        AutonMethods.driveTo(-27 ,  15 , 0.5);  // Go back to starting position
        AutonMethods.driveTo(16.5,1,0.5);       // Go to carousel
        AutonMethods.carouselOn(0.15, 5)        // Activate carousel
        AutonMethods.driveTo(3,17,0.5);         // Park
    
    

    }
}
