package org.firstinspires.ftc.teamcode;
//done other than testing?

import static java.lang.Math.abs;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import java.lang.Thread;



import java.util.Arrays;
/*
TODOs:
remove @overides
 */


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auton", group="Iterative Opmode")

public class Auton extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontWheel;
    private DcMotorEx rightFrontWheel;
    private DcMotorEx leftBackWheel;
    private DcMotorEx rightBackWheel;
    private DcMotorEx carouselMotor;
    private DcMotorEx elevatorMotor;
    private DcMotorEx intakeMotor;
    private Servo holderServo;
    private DcMotorEx transportMotor;
    final double sircumference = 0.299236638;
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double forwardBackward;
    double straifing;
    double turning;
    double x = -3.66;//meters (bottom right = 0)
    double y = .32;//meters//11.78097
    double orientation = 0;
    //double forwardBackward;
    //double straifing;
    //double turning;
    double distanceFromOrientation;
    final double disByWheelPerDegTurned = .00507680555;//the distance (in meters) travelled by each wheel per degree (of 360) turned
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
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        transportMotor = hardwareMap.get(DcMotorEx.class, "transportMotor");

        //TODO: add max and min values for servo
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontWheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotorEx.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        //Auton intake
        intakeMotor.setPower(1);
        
        


        
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }
    public void mecanum(){

      
        leftFrontPower    = forwardBackward - turning + straifing;
        rightFrontPower   = forwardBackward + turning - straifing;
        leftBackPower     = forwardBackward - turning - straifing;
        rightBackPower    = forwardBackward + turning + straifing;
        double[] powVals = {abs(leftFrontPower), abs(rightFrontPower), abs(leftBackPower), abs(rightBackPower)};
        Arrays.sort(powVals);
        if ((abs(leftFrontPower)) > 1 || (abs(rightFrontPower)) > 1 || (abs(leftBackPower) > 1) || (abs(rightBackPower) > 1))    {
            double maxPower = powVals[3];
            leftFrontPower    /= maxPower;
            rightFrontPower   /= maxPower;
            leftBackPower     /= maxPower;
            rightBackPower    /= maxPower;

        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftBackWheel.setPower(leftBackPower);
        rightBackWheel.setPower(rightBackPower);
        }
        else if(forwardBackward == 0 && straifing == 0 && turning == 0){
            leftFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }
    public void brake(){
     leftFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
     rightFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
     leftBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
     rightBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
   }
   
   public double encoderToMeters(){
      return ((rightBackWheel.getCurrentPosition()/560)*sircumference);
   }

    public void driveTo(double xDes, double yDes){
      
      rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      while (yDes > y){
        forwardBackward = 1;
        y += (encoderToMeters());
        mecanum();
      }
      forwardBackward = 0;
      rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      brake();
      while (yDes < y){
        forwardBackward = -1;
        y -= (encoderToMeters());
        mecanum();
      }
      forwardBackward = 0;
      rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
      brake();
      while (xDes < x){
        straifing = 1;
        x -= (encoderToMeters()/2);
        mecanum();
      }
      straifing = 0;
      rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
      brake();
      while (xDes > x){
        straifing = -1;
        x += (encoderToMeters()/2);
        mecanum();
      }
      rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
      straifing = 0;
      brake();
    }
    /*public void findDistanceFromOrientation(){//possibly a useless method with the new rotate method
      if (degree > orientation){
        distanceFromOrientation = degree - orientation;
      }
      else if (degree < orientation){
        distanceFromOrientation = orientation - degree;
      }
    }
*/
    public void rotate(double degree){//degree = DISTANCE (- or +) that you want to turn, not destination of turning
      if(degree > orientation){
        while (orientation < degree){
          turning = 1;
          mecanum();
          orientation += (encoderToMeters()/disByWheelPerDegTurned);
        }
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         
        turning = 0;
        brake();
      }
      else if (degree < orientation){
          while (orientation > degree){
            turning = -1;
            mecanum();
            orientation += (encoderToMeters()/disByWheelPerDegTurned);
          }
           
          rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//TODO: I think the syntax is wrong in this line and other identical ones
          turning = 0;
          brake();
      }
      //the below commented out code is the old rotate method which took a position between 0-360 as a parameter and moved to theat location
      //this encountered problems with having values above or below 360 and 0 which i did not have the time to fix
      /*findDistanceFromOrientation();
      if (degree > orientation){
        if (distanceFromOrientation > 180){
          while (orientation > degree + 1/*the preventWrongDegrees method will be implemented inside of these while loops* || orientation < degree - 1){
            turning = -1;
            mecanum();
            orientation += encoderToMeters/(-disByWheelPerDegTurned);
          }
          rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          turning = 0;
          brake();
        }
        if (distanceFromOrientation < 180){
          while (orientation > degree + 1 || orientation < degree - 1){
            turning = 1;
            mecanum();
            orientation += encoderToMeters/disByWheelPerDegTurned;
          }
          rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          turning = 0;
          brake();
        }
        //TODO: .4569125 is distance in meters that any wheels move during 90 degree turn
        //TODO: 0.00507680555 is the distance in meters that any wheels move during a 1 degree turn
        //TODO: I will finish dw
      }
      if (degree < orientation){
        if (distanceFromOrientation > 180){
          while (orientation > degree + 1 || orientation < degree - 1){
            turning = -1;
            mecanum();
            orientation += encoderToMeters/(-disByWheelPerDegTurned);
          }
          rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          turning = 0;
          brake();
        }
        if (distanceFromOrientation < 180){
          while (orientation > degree + 1 || orientation < degree - 1){
            turning = 1;
            mecanum();
            orientation += encoderToMeters/disByWheelPerDegTurned;
          }
          rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          turning = 0;
          brake();
        }
      }
      */
    }
    public void reorientate(){
      driveTo(-.3,.3);
      x = -3.66;
      y = 0;
      //orientation = 0; ?
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    public void start() {

        runtime.reset();
        
        while (runtime.seconds() < 10) { 
           carouselMotor.setPower(0.15);
        }
        carouselMotor.setPower(0);
        driveTo(-.32,.61);//rewrite driveTo mthd to take into account iruentation
        rotate(-90);
        driveTo(-3.05, -.1);
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.4



        


        //double elevatorHeight = elevatorMotor.getCurrentPosition(); TODO: test position values

        


        // Send calculated power to wheels
        

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //TODO: fix the following line to work with mecanum done?
        telemetry.addData("Motors", "leftFrontWheel (%.2f), rightFrontWheel (%.2f), leftBackWheel (%.2f), rightBackWheel (%.2f)", leftFrontWheel.getPower(), rightFrontWheel.getPower(), leftBackWheel.getPower(),rightBackWheel.getPower());

        /*
        gamepad1.dpad_direction
         *
         */

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

    public void wait (double seconds) {
        double startTime = runtime.seconds();
        while (runtime.seconds() - startTime < seconds) {
            continue;
        }
        /*
        while (true) {
            if (runtime.seconds() - startTime > seconds) {
                return;
                //break;
            }
        }
         */
    }
}
