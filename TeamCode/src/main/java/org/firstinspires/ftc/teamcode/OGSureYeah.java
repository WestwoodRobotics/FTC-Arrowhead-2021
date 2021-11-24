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

https://www.windows93.net/ 
https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit

TODO-LIST:
|-------------------------------------------------------|                     
|*make PID                                              |  
|*elevator (prepoprepositions) (constant levels)        |
|*Color Sensor implementation                           |                    
|*Intake stuff                                          |
|-------------------------------------------------------|
/*    chrisophe robot                            _____

             .--._
           .'""""-t--.
          /_. .-._ \  `.    __.-.
         :/o:  "o.` ; .'\.-" .--."-._
.       /;  ;       :/  _;. (    `-._\                                .-"
 "--._.: :  :s,     t,-"/  j"_.  _._\`\                   ___._.._.._/ _.
 _      \;  .--.  /  "-:  / 'o.` \.o`; ;              .-""    (      ""
/ "-.    :       :     ;,:   .  .$;  :/:_.---.-.      :: .-""""^._
  /       \_.gp.____.-j ;;  '   __   :_; _.--`-.t     ; Y        (
 :        ;`^$$P^"    ; : \    '  `  ; : ;       l--t"'.;_ ,_.-.  ;
 ;       /    \ (    :   \ t-.___   /  ;/  ,---=-:   \  ;o;  .o.` :
:     .-"      \ )   ;    `.;    """\ .'   .o' :o;    ; : ;        ;
;  :   ""--.._  `.  :     .'"-.      Y        .d.:    : ; Ts,-.  .-:  `--
  .'          "-._`.;   .'     \      \      .---,   ;  : ---. `   :
                                ;      \    :.---:   :   ;--     __;
                                :       ;--..  " )   ;  _:tjtjt-;
                                 ;      :    """"-.__L./ '^:;:^'     `-._
                            bug  :      :  /\         :
mount rush more
   .-----------------------===------------------------.
  :o  ______________________________________________  o:
  ;   :                                            :   ;
  `.  `.                                          .'  .'
   :   :                                          :   :
   `.  `.                                        .'  .'
    :   :   you don't have enough time           :   :
    `.  `.                                      .'  .'
     :   :                                      :   :
     `.  `.                                    .'  .'
      :   :                                    :   :
      `.  `.                                  .'  .'
       :   :__________________________________:   :
       `.               timebook                 .'
        '-------------|  |-----|  |--------------'
       .""""""""""""""|  |"""""|  |"""""""""""""".
       |  ()          '-----------'      o   ()  |
       |   ___________________________________   |
       |  :__|__|__|__|__|__|__|__|__|__|__|__:  |
       |  |___|_q_|_w_|_e_|_r_|_t_|_y_|_u_|_i_|_o_|_p_|[_| ] |
       |  |__|_|__|__|__|__|__|__|__|__|__|___|  |
       |  |___|__|__|__|__|__|__|__|__|__|____|  |
       |  |____|__|__|__|__|__|__|___|__|__|__|  |
       |  :___|__|___________________|__|__|__:  |
       |                                         |
       |             .-------------.             |
       |             |             |             |
       |             |             |             |
       |             |_____________|             |
       |             |             |             |
       |             |             |             |
       |             '.___________.'             |
       |                   ___                   |grp
       '-----------------------------------------'
       
                             O
                            (_)
                          _ )_( _
                        /`_) H (_`\
                      .' (  { }  ) '.
                    _/ /` '-'='-' `\ \_
                   [_.'   _,...,_   '._]
                    |   .:"`````":.   |
                    |__//_________\\__|
                     | .-----------. |
                     | |  .-"""-.  | |
                     | | /    /  \ | |
                     | ||-   <   -|| |
                     | | \    \  / | |
                     | |[`'-...-'`]| |
                     | | ;-.___.-; | |
                     | | |  |||  | | |
                     | | |  |||  | | |
                     | | |  |||  | | |
                     | | |  |||  | | |
                     | | |  |||  | | |
                     | | | _|||_ | | |
                     | | | >===< | | |
                     | | | |___| | | |
                     | | |  |||  | | |
                     | | |  ;-;  | | |
                     | | | (   ) | | |
                     | | |  '-'  | | |
                     | | '-------' | |
                jgs _| '-----------' |_
                   [= === === ==== == =]
                   [__--__--___--__--__]
                  /__-___-___-___-___-__\
                 `"""""""""""""""""""""""`
                  _____________________
|  _________________  |
| |              /  | |
| |       /\    /   | |
| |  /\  /  \  /    | |
| | /  \/    \/     | |
| |/             JO | |
| |_________________| |
|  __ __ __ __ __ __  |
| |__|__|__|__|__|__| |
| |__|__|__|__|__|__| |
| |__|__|__|__|__|__| |
| |__|__|__|__|__|__| |
| |__|__|__|__|__|__| |
| |__|__|__|__|__|__| |
|  ___ ___ ___   ___  |
| | 7 | 8 | 9 | | + | |
| |___|___|___| |___| |
| | 4 | 5 | 6 | | - | |
| |___|___|___| |___| |
| | 1 | 2 | 3 | | x | |
| |___|___|___| |___| |
| | . | 0 | = | | / | |
| |___|___|___| |___| |
|_____________________|
*/
/*public class PushbotAutoDriveByEncoder_Linear extends LinearOpMode {*/
public class Auton extends LinearOpMode {


  //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
  private ElapsedTime     runtime = new ElapsedTime();
  private ElapsedTime     servoTimer = new ElapsedTime();

  static final double     COUNTS_PER_MOTOR_REV          = 560 ;    // eg: TETRIX Motor Encoder
  static final double     DRIVE_GEAR_REDUCTION          = 20 ;     // This is < 1.0 if geared UP
  static final double     WHEEL_SIRCONFERENCE_INCHES    = 11.78097 ;     // For figuring circumference
  static final double     COUNTS_PER_INCH             = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_SIRCONFERENCE_INCHES);
  static final double     DRIVE_SPEED                 = 0.6;
  static final double     TURN_SPEED                  = 0.5;

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
        if (scannerPosition == 1) {
          servoServingMoverMovement(PLACEHOLDER_TIME, PLACEHOLDER_POSITION);
        }
        else if (scannerPosition == 2) {
          servoServingMoverMovement(PLACEHOLDER_TIME, PLACEHOLDER_POSITION);
        }
        else if (scannerPosition == 3) {
          servoServingMoverMovement(PLACEHOLDER_TIME, PLACEHOLDER_POSITION);
        }

        

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
  public void encoderDrive(double speed, double forwardBackwardInches, double sidewaysInches, double timeoutS) {
    if (forwardBackwardInches > 0){//if movement is set to forwards
      setAllTargets(forwardBackwardInches);//sets the targets for all wheels
      setAllPower(speed);//sets the power
      runToPos();//turns on run to position for all wheels
      resetEncodersAfterMovementComplete();//waits until movement is complete, then resets all encoders
    } else if (forwardBackwardInches < 0){//if movement is set to backwards
      setAllTargets(forwardBackwardInches);
      setAllPower(-speed);                                     // apparently RUN_TO_POSITION moves the thing to the right place, accounts for the negative already. May have to change this
      runToPos();
      resetEncodersAfterMovementComplete();
    } 
    if (sidewaysInches > 0){//if movement is right
      mecanumTargets(sidewaysInches);//sets the targets according to mecanum for all wheels
      straif(power);//sets the power according to mecanum
      runToPos();//turns on run to position for all wheels
      resetEncodersAfterMovementComplete();// reset all econders once movement is complete
    } else if (sidewaysInches < 0){
      mecanumTargets(sideWaysInches);
      straif(-power);
      runToPos;
      resetEncodersAfterMovementComplete();
    }
    
    public void elevatorLevel(int level) {
      


      
     while (opModeIsActive() && (runtime.seconds() < timeoutS) &&  (leftFrontMotor.isBusy() || rightFrontMotor.isBusy() || leftBackMotor.isBusy() || rightBackMotor.isBusy())) {

      // Display it for the driver.
      telemetry.addData("Parth2",  "Running at %7d :%7d", leftFrontMotor.getCurrentPosition(),rightFrontMotor.getCurrentPosition());
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
  }
}
