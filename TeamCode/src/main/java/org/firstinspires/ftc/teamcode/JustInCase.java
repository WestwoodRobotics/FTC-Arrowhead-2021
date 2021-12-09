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

@Autonomous(name="Emergency Backup", group="Iterative Opmode")


public class JustInCase extends LinearOpMode {


    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     servoTimer = new ElapsedTime();
    private ElapsedTime     PIDTime = new ElapsedTime();
    private ElapsedTime     intakeTime = new ElapsedTime();


    static final int     COUNTS_PER_MOTOR_REV          = 560 ;
    static final int     DRIVE_GEAR_REDUCTION          = 20 ;
    static final double     WHEEL_SIRCONFERENCE_INCHES    = 11.78097 ;
    static final double     COUNTS_PER_INCH             = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /WHEEL_SIRCONFERENCE_INCHES;
    static final double     DRIVE_speade                 = 0.6;
    static final double     TURN_speade                  = 0.5;
    static final double     ELEVATOR_GEAR_RATIO         = 50.9;
    static final double     COUNTS_PER_ELEVATOR_REV     = 1425;
    static final double     MAX_ELEVATOR_CAPABLITY      = 123;
    static final double     ELEVATOR_SPOOL_DIAMETER_INCHES = 1.49606;
    static final double     ELE_TICKS_PER_INCH   = (ELEVATOR_GEAR_RATIO*COUNTS_PER_ELEVATOR_REV)/ELEVATOR_SPOOL_DIAMETER_INCHES; //someone check my math pls
    static final int     TICKS_PER_REVOLUTION          = DRIVE_GEAR_REDUCTION*COUNTS_PER_MOTOR_REV;


    private DcMotorEx leftFrontWheel;
    private DcMotorEx rightFrontWheel;
    private DcMotorEx leftBackWheel;
    private DcMotorEx rightBackWheel;
    private DcMotorEx carouselMotor;
    private CRServo holderServo;
    private DcMotorEx elevatorMotor;
    private DcMotorEx intakeMotor;



    // P function
    final double kP = 0.1;// placeholder value that needs to be tested
    final double kI = 1.0;
    final double kD = 0.1;
    double errorSum = 0.0;
    double error = 0.0;
    double previousError = 0.0;
    double maxPotentialOvershoot = .01;
    final double distancePerDegTurned = 0.01665618;
    double mecanumPower = 0.8;
    double destinationFeet;
    double currentPositionFeet;
    double orientation = 0;





    public void runOpMode() {



        telemetry.addData("Status", "Initialized");

        leftFrontWheel  = hardwareMap.get(DcMotorEx.class, "leftFrontWheel");
        rightFrontWheel = hardwareMap.get(DcMotorEx.class, "rightFrontWheel");
        leftBackWheel = hardwareMap.get(DcMotorEx.class, "leftBackWheel");
        rightBackWheel = hardwareMap.get(DcMotorEx.class, "rightBackWheel");
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        holderServo = hardwareMap.get(CRServo.class, "holderServo");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        leftFrontWheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotorEx.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
        holderServo.setDirection(CRServo.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontWheel.setPositionPIDFCoefficients(0.5);
        rightFrontWheel.setPositionPIDFCoefficients(0.5);
        leftBackWheel.setPositionPIDFCoefficients(0.5);
        rightBackWheel.setPositionPIDFCoefficients(0.5);
        elevatorMotor.setPositionPIDFCoefficients(0.5);

        telemetry.addData("Status", "Initialized");




        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Parth0",  "Starting at %7d :%7d");

        telemetry.update();
        // telemetry.addData("if your gonna slam into the wall you're always gonna get where you need to go");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while (runtime.milliseconds() < 200)
        {
            leftFrontWheel.setPower(.6);
            leftBackWheel.setPower(-.6);
            rightFrontWheel.setPower(-.6);
            rightBackWheel.setPower(.6);
        }
        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);
        while (runtime.milliseconds() < 1100){
            carouselMotor.setPower(.15);
        }
        carouselMotor.setPower(0);
        while (runtime.milliseconds() < 200){
            leftFrontWheel.setPower(-.6);
            leftBackWheel.setPower(.6);
            rightFrontWheel.setPower(.6);
            rightBackWheel.setPower(-.6);
        }
        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);



        telemetry.addData("Parth", "Complete");
        telemetry.update();
    }


}















