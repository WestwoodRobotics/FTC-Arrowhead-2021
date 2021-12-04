package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@Autonomous(name="Our Marvelous Autonomous", group="Iterative Opmode")


public class OGSureYeah extends LinearOpMode {


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
    private Servo holderServo;
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
        holderServo = hardwareMap.get(Servo.class, "holderServo");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            
        leftFrontWheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotorEx.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
        holderServo.setDirection(Servo.Direction.FORWARD);
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
        
 

        telemetry.addData("Parth", "Complete");
        telemetry.update();
}
    


    public void setAllPower(double power){
        rightFrontWheel.setPower(power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        leftBackWheel.setPower(power);
    }
    //sets all the motors according to mecanum and to a specified power value
    public void straif(double power){
        rightFrontWheel.setPower(-power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(power);
        leftBackWheel.setPower(-power);
    }
    //sets the targets for all wheels to the same and accounts for sriconference, gear reduction,  and ticks per rev
    //useful for setting forwards or backwards targets
    public void setAllTargets(double targetDist){
        int targetValue = (int) Math.round(targetDist/WHEEL_SIRCONFERENCE_INCHES*12);
        rightFrontWheel.setTargetPosition((targetValue)*TICKS_PER_REVOLUTION);
        leftFrontWheel.setTargetPosition((targetValue)*TICKS_PER_REVOLUTION);
        rightBackWheel.setTargetPosition((targetValue)*TICKS_PER_REVOLUTION);
        leftBackWheel.setTargetPosition((targetValue)*TICKS_PER_REVOLUTION);
    }
    // sets all the targets according to mecanum and accounrs for sri cfonfrince, gear reduction, and ticks per rev
    //useful for setting sideways targets
    public void mecanumTargets(double targetDist){
        int sirTarget = (int) Math.round((targetDist/WHEEL_SIRCONFERENCE_INCHES)*TICKS_PER_REVOLUTION*12);
        rightFrontWheel.setTargetPosition(-sirTarget);
        leftFrontWheel.setTargetPosition(sirTarget);
        rightBackWheel.setTargetPosition(sirTarget);
        leftBackWheel.setTargetPosition(-sirTarget);
    }
    public void runToPos(){
        
        while (Math.abs(leftFrontWheel.getPower()) > 0 && Math.abs(rightFrontWheel.getPower()) > 0 && Math.abs(leftBackWheel.getPower()) > 0 && Math.abs(rightBackWheel.getPower()) > 0){
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
    }
   
    //this method resets the encoders after waiting until movement is complete
    //implement in order to prevent false encoder readings, which throw off future movement
    public void resetEncodersAfterMovementComplete(){
        while (leftFrontWheel.isBusy() || rightFrontWheel.isBusy() || leftBackWheel.isBusy() || rightBackWheel.isBusy()){
            //waits until the motors are done
        }
        //TODO:put setZeroPowerBehaiviour(BRAKE thing)
        //once the motors are done moving, this method resets the encoders
        if (!(leftFrontWheel.isBusy() || rightFrontWheel.isBusy() || leftBackWheel.isBusy() || rightBackWheel.isBusy())){
            leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void servoServingMoverMovement(double timerTime, double servoServe) {
        while (servoTimer.seconds() < timerTime) {
        }
        holderServo.setPosition(servoServe);
        while (servoTimer.seconds() < (timerTime + 5)) {
        }
        holderServo.setPosition(0.0);

    }
        public void mecanumTurning(double power){
        rightFrontWheel.setPower(-power);
        leftFrontWheel.setPower(power);
        rightBackWheel.setPower(-power);
        leftBackWheel.setPower(power);
    }

    public void encoderDrive( double forwardBackwardFoots, double sidewaysFoots, double timeoutS) {
        if (forwardBackwardFoots > 0) {//if movement is set to forwards
            setAllTargets(forwardBackwardFoots);//sets the targets for all wheels//setAllPower(PID(speade))
            destinationFeet = forwardBackwardFoots;
            //while(currentPositionFeet > forwardBackwardFoots -.05 && < forwardBackwardsFoots + .05){
            //    setAllPower(mecanumPower);//sets the power
            //}
            while (leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy()){                     runToPos();                     }//turns on run to position for all wheels
            resetEncodersAfterMovementComplete();//waits until movement is complete, then resets all encoders
        } else if (forwardBackwardFoots < 0) {//if movement is set to backwards
            setAllTargets(forwardBackwardFoots);
            //while(currentPositionFeet > forwardBackwardFoots +.05 && < forwardBackwardsFoots - .05){
            //    setAllPower(-mecanumPower);//sets the power
            //}                                   // apparently RUN_TO_POSITION moves the thing to the right place, accounts for the negative already. May have to change this
            while (leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy()){                     runToPos();                     }
            resetEncodersAfterMovementComplete();
        }
        if (sidewaysFoots > 0) {//if movement is right
            mecanumTargets(sidewaysFoots);//sets the targets according to mecanum for all wheels
            //while (currentPositionFeet > sidewaysFoots - .05 && currentPositionFeet < sidewaysFoots + .05){
            //    straif(mecanumPower);//sets the power according to mecanum
            //}
            while (leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy()){                     runToPos();                     }//turns on run to position for all wheels
            resetEncodersAfterMovementComplete();// reset all econders once movement is complete
        } else if (sidewaysFoots < 0) {
            mecanumTargets(sidewaysFoots);
            //while(currentPositionFeet > forwardBackwardFoots +.05 && < forwardBackwardsFoots - .05){
            //    setAllPower(-mecanumPower);//sets the power
            //}
            while (leftFrontWheel.isBusy() && rightFrontWheel.isBusy() && leftBackWheel.isBusy() && rightBackWheel.isBusy()){                     runToPos();                     }
            resetEncodersAfterMovementComplete();
        }
    }
    public double encoderToMeters(){
    return ((leftFrontWheel.getCurrentPosition()/560)*WHEEL_SIRCONFERENCE_INCHES);
    }

   public void theParth() {
        	encoderDrive(4, 2, 1);
        	rotate(90);
        	encoderDrive(2, -1, 1);     
        
    }
        
    public void intakeOn(boolean on, double time,  double forwardBackwardFoots, double sidewaysFoots, double timeoutS){
        if (on){
            intakeMotor.setPower(.7);
        }
        intakeTime.reset();
        while(intakeTime.milliseconds() < time){
            encoderDrive( forwardBackwardFoots,  sidewaysFoots,  timeoutS);
        }
        intakeMotor.setPower(0);
    }
        
        public void turnTargets (double degree){
            int numberOfDistance = (int) Math.round((distancePerDegTurned*degree));
            leftFrontWheel.setTargetPosition(numberOfDistance);
            rightFrontWheel.setTargetPosition(-numberOfDistance);
            leftBackWheel.setTargetPosition(numberOfDistance);
            rightBackWheel.setTargetPosition(-numberOfDistance);
        }
        
        public void rotate(double degree) {//degree = DISTANCE (- or +) that you want to turn, not destination of turning
            
            if (degree > orientation) {
                while (orientation < degree + 1) {
                    turnTargets(degree);
                    mecanumTurning(.3);
                    runToPos();
                    orientation += (encoderToMeters() / distancePerDegTurned);
                }
                setAllPower(0);
                resetEncodersAfterMovementComplete();

            }
             else if (degree < orientation) {
                while (orientation > degree - 1) {
                    turnTargets(degree);
                    mecanumTurning(.3);
                    
                    runToPos();
                    
                    orientation += (encoderToMeters() / distancePerDegTurned);
                }
                setAllPower(0);

                resetEncodersAfterMovementComplete();
            }
            
            theParth();


            while (opModeIsActive() && (leftFrontWheel.isBusy() || rightFrontWheel.isBusy() || leftBackWheel.isBusy() || rightBackWheel.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Parth2", "Running at %7d :%7d", leftFrontWheel.getCurrentPosition(), rightFrontWheel.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontWheel.setPower(0);
            rightBackWheel.setPower(0);
            leftBackWheel.setPower(0);
            rightFrontWheel.setPower(0);
            // Turn off RUN_TO_POSITION
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
}

    
            











