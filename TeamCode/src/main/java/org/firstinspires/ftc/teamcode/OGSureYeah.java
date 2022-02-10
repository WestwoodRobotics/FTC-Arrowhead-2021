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
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx carouselMotor;
    private CRServo holderServo;
    private DcMotorEx elevatorMotor;
    private DcMotorEx intakeMotor;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontMotor  = hardwareMap.get(DcMotorEx.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBackMotor");
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        holderServo = hardwareMap.get(CRServo.class, "holderServo");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        //TODO: add max and min values for servo
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
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
        // Setup a variable for each drive Motor to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double carouselPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double forwardBackward = gamepad1.left_stick_y;
        double straifing =  gamepad1.left_stick_x;
        double turning = gamepad1.right_stick_x;


        //double elevatorHeight = elevatorMotor.getCurrentPosition(); TODO: test position values

        leftFrontPower    = 0.6*(forwardBackward + turning - straifing);
        rightFrontPower   = 0.6*(forwardBackward - turning + straifing);
        leftBackPower     = 0.6*(forwardBackward + turning + straifing);
        rightBackPower    = 0.6*(forwardBackward - turning - straifing);
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
            holderServo.setPower(0.6);
            }
        else if(gamepad2.left_trigger > 0){
            holderServo.setPower(-0.6);
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
            elevatorMotor.setPower(0.6);
        }
        else if (gamepad2.dpad_down) {
            elevatorMotor.setPower(-0.6);
        }
        else if (gamepad2.dpad_left) {
            elevatorMotor.setPower(-0.3);
        }
        else if (gamepad2.dpad_right) {
            elevatorMotor.setPower(0.3);
        }
        else {
            elevatorMotor.setPower(0.0);
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

        // Send calculated power to Motors
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);

        telemetry.addData("ok ", "buddy: " + runtime.toString());
    
        telemetry.addData("OK buddy", "leftFrontBuddy (%.2f), rightFrontBuddy (%.2f), leftBackOK (%.2f), rightBackOK (%.2f)", leftFrontMotor.getPower(), rightFrontMotor.getPower(), leftBackMotor.getPower(),rightBackMotor.getPower());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    public void stop() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        carouselMotor.setPower(0);
        rightBackMotor.setPower(0);
        

    }

    public void wait(double seconds) {
        double startTime = runtime.seconds();
        while (runtime.seconds() - startTime < seconds) {
            continue;
        }

    }
}









