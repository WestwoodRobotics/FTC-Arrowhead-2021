package org.firstinspires.ftc.teamcode;
//done other than testing?

import static java.lang.Math.abs;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Arrays;


@TeleOp(name="Teletubbies", group="Iterative Opmode")

public class Teletubbies extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontWheel;   // Front Left Wheel
    private DcMotorEx rightFrontWheel;  // Front Right Wheel
    private DcMotorEx leftBackWheel;    // Back Left Wheel
    private DcMotorEx rightBackWheel;   // Back Right Wheel
    private DcMotorEx carouselMotor;    // Carousel Wheel
    private CRServo holderServo;        // Freight Receptacle
    private DcMotorEx elevatorMotor;    // Elevator Lift
    private DcMotorEx intakeMotor;      // Intake Wheels
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double carouselPower;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontWheel  = hardwareMap.get(DcMotorEx.class, "leftFrontWheel");
        rightFrontWheel = hardwareMap.get(DcMotorEx.class, "rightFrontWheel");
        leftBackWheel = hardwareMap.get(DcMotorEx.class, "leftBackWheel");
        rightBackWheel = hardwareMap.get(DcMotorEx.class, "rightBackWheel");
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        holderServo = hardwareMap.get(CRServo.class, "holderServo");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        //TODO: add max and min values for servo
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontWheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotorEx.Direction.FORWARD);
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
        holderServo.setDirection(CRServo.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry



        double forwardBackward = -gamepad1.left_stick_y;
        double straifing =  gamepad1.left_stick_x;
        double turning = gamepad1.right_stick_x;

        leftFrontPower    = 0.8*(forwardBackward - turning - straifing);
        rightFrontPower   = 0.8*(forwardBackward + turning + straifing);
        leftBackPower     = 0.8*(forwardBackward + turning - straifing);
        rightBackPower    = 0.8*(forwardBackward - turning + straifing);
        
        double[] powVals = {abs(leftFrontPower), abs(rightFrontPower), abs(leftBackPower), abs(rightBackPower)};
        Arrays.sort(powVals);
        
        if ((abs(leftFrontPower)) > 1 || (abs(rightFrontPower)) > 1 || (abs(leftBackPower) > 1) || (abs(rightBackPower) > 1))    {
            double maxPower = powVals[3];
            leftFrontPower    /= maxPower;
            rightFrontPower   /= maxPower;
            leftBackPower     /= maxPower;
            rightBackPower    /= maxPower;
        }
        

        if(gamepad1.a){
            carouselMotor.setPower(0.15);
        }
        else if (gamepad1.y){
            carouselMotor.setPower(-0.15);

        }
        else if(gamepad1.b){
            carouselMotor.setPower(0);
        }


        if(gamepad2.right_trigger > 0) {
            holderServo.setPower(0.65);
            }
        else if(gamepad2.left_trigger > 0){
            holderServo.setPower(-0.65);
            }
        else{ holderServo.setPower(0.0);}
        
        if(gamepad2.a) {
            intakeMotor.setPower(0.8);
            }
        else if(gamepad2.b) {
            intakeMotor.setPower(0);
            }
        else if(gamepad2.y) {
            intakeMotor.setPower(-0.8);
            }
            
        
        if (gamepad2.dpad_up) {
            elevatorMotor.setPower(0.75);
            // Elevator Up (Fast)
        }
        else if (gamepad2.dpad_down) {
            elevatorMotor.setPower(-0.75);
            // Elevator Down (Fast)
        }
        else if (gamepad2.dpad_left) {
            elevatorMotor.setPower(-0.25);
            // Elevator Down (Slow)
        }
        else if (gamepad2.dpad_right) {
            elevatorMotor.setPower(0.25);
            // Elevator Up (Slow)
        }
        else {
            elevatorMotor.setPower(0.0);
            // Cease Motion
        }
        
        

        if (gamepad1.left_trigger > 0) {
            leftFrontPower = 0.3;
            leftBackPower = 0.3;
            rightFrontPower = -0.3;
            rightBackPower = -0.3;
        }
        else if (gamepad1.right_trigger > 0) {
            leftFrontPower = -0.3;
            leftBackPower = -0.3;
            rightFrontPower = 0.3;
            rightBackPower = 0.3;
        }

        // Send calculated power to wheels
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftBackWheel.setPower(leftBackPower);
        rightBackWheel.setPower(rightBackPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    
        telemetry.addData("Motors", "leftFrontWheel (%.2f), rightFrontWheel (%.2f), leftBackWheel (%.2f), rightBackWheel (%.2f)", leftFrontWheel.getPower(), rightFrontWheel.getPower(), leftBackWheel.getPower(),rightBackWheel.getPower());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void stop() {
        leftFrontWheel.setPower(0);
        rightFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        carouselMotor.setPower(0);
        rightBackWheel.setPower(0);
        

    }

    public void wait(double seconds) {
        double startTime = runtime.seconds();
        while (runtime.seconds() - startTime < seconds) {
            continue;
        }

    }
}








