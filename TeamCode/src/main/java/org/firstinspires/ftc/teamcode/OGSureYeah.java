package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="Good Autonomous", group="Iterative Opmode")



public class OGSureYeah extends LinearOpMode {

    
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx carouselMotor;
    private Servo holderServo;
    private DcMotorEx elevatorMotor;
    private DcMotorEx intakeMotor;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        
        

        
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBackMotor");
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        holderServo = hardwareMap.get(Servo.class, "holderServo");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        
        leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
        holderServo.setDirection(Servo.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        
        AutoMethods AutonMethods = new AutoMethods(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, carouselMotor, holderServo, elevatorMotor, intakeMotor);

        
        // put setdirections back here if it breaks
        
        
        // ArrowheadPurelyOriginalWorkDoNotCopyTensorFlowMethods.something something scan file

        ottosMethods.setAllPositionPIDFCoefs(0.5); 
        ottosMethods.setMecTargets(5);
        ottosMethods.runUsingPositions();
        

        ottosMethods.driveToPosition( 27 , -15 , 600);  // Go to Alliance shipping hub

        ottosMethods.driveToPosition(-27 ,  15 , 600);  // Go back to starting position
        ottosMethods.driveToPosition(16.5,1,600);       // Go to carousel
        ottosMethods.carouselOn(0.15, 5);        // Activate carousel
        ottosMethods.driveToPosition(3,17,0.5);         // Park
    
    

    }
}


