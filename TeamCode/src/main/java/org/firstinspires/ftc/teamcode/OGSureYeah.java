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

/*
TODO-LIST:
|-------------------------------------------------------|
|                                                       |
|*elevator (prepoprepositions) (constant levels)        |
|*Color Sensor implementation <-- not happening         |
|*Intake stuff                                          |
|*find out how many times code runs per/sec             |
|-------------------------------------------------------|
*/

/*public class PushbotAutoDriveByEncoder_Linear extends LinearOpMode {*/
public class Auton extends LinearOpMode {


    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     servoTimer = new ElapsedTime();
    private ElapsedTime     PIDTime = new ElapsedTime(ElapsedTime.resolution milliseconds);

    static final double     COUNTS_PER_MOTOR_REV          = 560 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION          = 20 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_SIRCONFERENCE_INCHES    = 11.78097 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH             = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_SIRCONFERENCE_INCHES);
    static final double     DRIVE_SPEED                 = 0.6;
    static final double     TURN_SPEED                  = 0.5;
    static final double     ELEVATOR_GEAR_RATIO         = 50.9;
    static final double     COUNTS_PER_ELEVATOR_REV     = 1425;
    static final double     MAX_ELEVATOR_CAPABLITY      = TBD;
    static final double     ELE_TICKS_PER_INCH   = (ELEVATOR_GEAR_RATIO*COUNTS_PER_ELEVATOR_REV)/eleSpoolDiameterInches //someone check my math pls

    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private DcMotorEx carouselMotor;
    private Servo holderServo;
    private DcMotorEx elevatorMotor;
    private DcMOtorEx intakeMotor;
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
    final double eleSpoolDiameterInches = 1.49606;
/*
    public void PID(){
        while (opModeIsActive()){
            PIDTime.reset();
            errorSum = 0; // adlsfjadsklfajs
            while ((destinationFeet > currentPositionFeet-.05) && (destinationFeet < currentPositionFeet+.05)){ // change 0.05 depending on preciseness
                previousError = error;//start of P
                currentPositionFeet = ((leftFrontMotor.getCurrentPosition() / COUNTS_PER_INCH) * 12);
                double distAtEighty = .8*destinationFeet; //maybe change it to a constant # of feet instead of 0.8 mabye
                double icurrentPositionFeet;
                error = (destinationFeet - currentPositionFeet)/maxPotentialOvershoot;
                // may exceed 1.0 motor power. Needs to fit between 0 and 1
                //start of I
                while (currentPositionFeet > distAtEighty){
                    icurrentPositionFeet = currentPositionFeet - distAtEighty;
                    if(PIDTime.milliseconds()%1 = 0){
                        errorSum += error * PIDTime.milliseconds();
                    }
                }
                //start of D
                double errorRate * (error - lastError) / PIDTime;
                //calculates the output
                mecanumPower = (kP * error)/maxPotentialOvershoot) + (kI * errorSum;) + (kD * errorRate));
            }
            mecanumPower = 0;
            resetEncodersAfterMovementComplete()
            currentPositionFeet = ((leftFrontMotor.getCurrentPosition() / COUNTS_PER_INCH) * 12);
        }
    }

    // I function

    // placeholder
*/
    leftFrontMotor.setPIDFCoefficients(1,2,3);
    rightFrontMotor.setPIDFCoefficients(1,2,3);
    leftBackMotor.setPIDFCoefficients(1,2,3);
    rightBackMotor.setPIDFCoefficients(1,2,3);
    
    elevatorMotor.setPIDFCoefficients(1,1,1);




    public void runOpMode() {
        init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Parth0",  "Starting at %7d :%7d",

                telemetry.update();
        // telemetry.addData("if your gonna slam into the wall you're always gonna get where you need to go");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Servo movement based on where the duck/custom game object is on the bar code
/*
        if (scannerPosition == 1) {
            //servoServingMoverMovement(PLACEHOLDER_TIME, PLACEHOLDER_POSITION);
            elevatorMotor.setPower(.35);
            elevatorMotor.setTargetPosition(numInches*ELE_TICKS_PER_INCH)
            elevatorMotor.runToPos();
        }
        else if (scannerPosition == 2) {
            //servoServingMoverMovement(PLACEHOLDER_TIME, PLACEHOLDER_POSITION);
            elevatorMotor.setPower(.35);
            elevatorMotor.setTargetPosition(numInches*ELE_TICKS_PER_INCH)
            elevatorMotor.runToPos();
        }
        else {
            //servoServingMoverMovement(PLACEHOLDER_TIME, PLACEHOLDER_POSITION);
            elevatorMotor.setPower(.35);
            elevatorMotor.setTargetPosition(numInches*ELE_TICKS_PER_INCH)
            elevatorMotor.runToPos();
        }
*/


        telemetry.addData("Parth", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    // sets all the powers to the same value
    public void setAllPower(double power){
        rightFrontMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        leftBackMotor.setPower(power);
    }
    //sets all the motors according to mecanum and to a specified power value
    public void straif(double power){
        rightFrontMotor.setPower(-power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        leftBackMotor.setPower(-power);
    }
    //sets the targets for all wheels to the same and accounts for sriconference, gear reduction,  and ticks per rev
    //useful for setting forwards or backwards targets
    public void setAllTargets(double targetDist){
        rightFrontMotor.setTargetPosition((targetDist/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        leftFrontMotor.setTargetPosition((targetDist/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        rightBackMotor.setTargetPosition((targetDist/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
        leftBackMotor.setTargetPosition((targetDist/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION);
    }
    // sets all the targets according to mecanum and accounrs for sri cfonfrince, gear reduction, and ticks per rev
    //useful for setting sideways targets
    public void mecanumTargets(double targetDist){
        rightFrontMotor.setTargetPosition(-((targetDist/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftFrontMotor.setTargetPosition(((targetDist/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        rightBackMotor.setTargetPosition(((targetDist/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
        leftBackMotor.setTargetPosition(-((targetDist/WHEEL_SIRCONFERENCE_INCHES)*COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION));
    }
    public void runToPos(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private ElapsedTime intakeTime = new ElapsedTime(ElapsedTime.resolution seconds);
    public void intakeOn(boolean on, double time, double speed, double forwardBackwardFoots, double sidewaysFoots, double timeoutS){
        if (on){
            intakeMotor.setPower(.7);
        }
        intakeTime.reset();
        while(intakeTime < time){
            encoderDrive(double speed, double forwardBackwardFoots, double sidewaysFoots, double timeoutS);
        }
        intakeMotor.setPower(0);
    }
    //this method resets the encoders after waiting until movement is complete
    //implement in order to prevent false encoder readings, which throw off future movement
    public void resetEncodersAfterMovementComplete(){
        while (leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftBackMotor.isBusy() || rightBackMotor.isBusy()){
            //waits until the motors are done
        }
        //TODO:put setZeroPowerBehaiviour(BRAKE thing)
        //once the motors are done moving, this method resets the encoders
        if (!(leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftBackMotor.isBusy() || rightBackMotor.isBusy())){
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
}
    public void mecanumTurning(double power){
        rightFrontMotor.setPower(-power);
        leftFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
        leftBackMotor.setPower(power);
    }
    public void encoderDrive(double speed, double forwardBackwardFoots, double sidewaysFoots, double timeoutS) {
        if (forwardBackwardFoots > 0) {//if movement is set to forwards
            setAllTargets(forwardBackwardFoots);//sets the targets for all wheels//setAllPower(PID(speed))
            destinationFeet = forwardBackwardFoots;
            //while(currentPositionFeet > forwardBackwardFoots -.05 && < forwardBackwardsFoots + .05){
            //    setAllPower(mecanumPower);//sets the power
            //}
            runToPos();//turns on run to position for all wheels
            resetEncodersAfterMovementComplete();//waits until movement is complete, then resets all encoders
        } else if (forwardBackwardFoots < 0) {//if movement is set to backwards
            setAllTargets(forwardBackwardFoots);
            //while(currentPositionFeet > forwardBackwardFoots +.05 && < forwardBackwardsFoots - .05){
            //    setAllPower(-mecanumPower);//sets the power
            //}                                   // apparently RUN_TO_POSITION moves the thing to the right place, accounts for the negative already. May have to change this
            runToPos();
            resetEncodersAfterMovementComplete();
        }
        if (sidewaysFoots > 0) {//if movement is right
            mecanumTargets(sidewaysFoots);//sets the targets according to mecanum for all wheels
            //while (currentPositionFeet > sidewaysFoots - .05 && currentPositionFeet < sidewaysFoots + .05){
            //    straif(mecanumPower);//sets the power according to mecanum
            //}
            runToPos();//turns on run to position for all wheels
            resetEncodersAfterMovementComplete();// reset all econders once movement is complete
        } else if (sidewaysFoots < 0) {
            mecanumTargets(sidewaysFoots);
            //while(currentPositionFeet > forwardBackwardFoots +.05 && < forwardBackwardsFoots - .05){
            //    setAllPower(-mecanumPower);//sets the power
            //}
            runToPos;
            resetEncodersAfterMovementComplete();
        }
        public void turnTargets ( double degree){
            leftFrontMotor.setTargetPosition(distancePerDegTurned * degree);
            rightFrontMotor.setTargetPosition(-distancePerDegTurned * degree);
            leftBackMotor.setTargetPosition(distancePerDegTurned * degree);
            rightBackMotor.setTargetPostion(-distancePerDegTurned * degree);
        }
        public void rotate ( double degree)
        {//degree = DISTANCE (- or +) that you want to turn, not destination of turning

            if (degree > orientation) {
                while (orientation < degree + 1) {
                    turnTargets(degree);
                    mecanumTurning(.3);
                    runToPos();
                    orientation += (encoderToMeters() / disByWheelPerDegTurned);
                }
                setAllPower(0);
                resetEncodersAfterMovementComplete();


            } else if (degree < orientation) {
                while (orientation > degree - 1) {
                    turnTargets(degree);
                    mecanumTurning(.3);
                    runToPos();
                    orientation += (encoderToMeters() / disByWheelPerDegTurned);
                }
                setAllPower(0);

                resetEncodersAfterMovementComplete();
            }


            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftBackMotor.isBusy() || rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Parth2", "Running at %7d :%7d", leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
            

    public void theParth() {
        encoderDrive(.7, 4, 0, 1);
        rotate(90);
        
        }


