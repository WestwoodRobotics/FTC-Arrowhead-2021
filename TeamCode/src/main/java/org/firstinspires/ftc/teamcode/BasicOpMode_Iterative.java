/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;
//done other than testing?
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;
/*
TODOs:
remove @overides
 */
*/
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

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")

public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftBackWheel;
    private DcMotor rightBackWheel;
    private DcMotor elevatorMotor;
    private DcMotor intakeMotor;
    private DcMotor carouselMotor;
    private Servo holderServo;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontWheel  = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontWheel = hardwareMap.get(DcMotor.class, "right_front");
        leftBackWheel = hardwareMap.get(DcMotor.class, "left_back");
        rightBackWheel = hardwareMap.get(DcMotor.class, "right_back");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        carouselMotor = hardwareMap.get(DcMotor.class, "carousel");
        holderServo = hardwareMap.get(Servo.class, "holder")//TODO:wrong?
        //TODO: add max and min values for servo
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotor.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);

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
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double elevatorPower;
        double intakePower;//TODO: servo power? and intake toggleablle
        double carouselPower;
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double forwardBackward = -gamepad1.left_stick_y;
        double straifing =  gamepad1.left_stick_x;
        double turning = gamepad1.right_stick_x;


        //double elevatorHeight = elevatorMotor.getCurrentPosition(); TODO: test position values
        int elevatorPos = 1;
        int seudoPID = 1;
        /*if (gamepad1.dpad_up == true){
            seudoPID = 1;
            elevatorMotor.setTargetPosition(1/TODO:put target position number*);
            while (elevatorMotor.getCurrentPosition() != /TODO:put target posotion number){
                while (elevatorMotor.getCurrentPosition() < /*TODO: put target postition) {
                    elevatorMotor.setPower(1/seudoPID);//TODO:dont tell me this is getting to PID
                    suedoPID++;
                }
                while (elevatorMotor.getCurrentPosition() > /*TODO: put target position){
                    elevatorMotor.setPower(-1/seudoPID);
                    seudoPID++;
                }
                if(seudoPID > 10){
                    break;
                }
            }
            elevatorMotor.setPower(0);

        }
        if (gamepad1.dpad_right == true){
            seudoPID = 1;
            elevatorMotor.setTargetPosition(1/*TODO:put target position number);
            while (elevatorMotor.getCurrentPosition() != /*TODO:put target posotion number){
                while (elevatorMotor.getCurrentPosition() < /*TODO: put target postition) {
                    elevatorMotor.setPower(1/seudoPID);//TODO:dont tell me this is getting to PID
                    seudoPID++;
                }
                while (elevatorMotor.getCurrentPosition() > /*TODO: put target position){
                    elevatorMotor.setPower(-1/seudoPID);
                    seudoPID++;//pstupid
                }
                if(seudoPID > 10){
                    break;
                }
            }
            elevatorMotor.setPower(0);
        }
        if (gamepad1.dpad_left == true){
            seudoPID = 1;
            elevatorMotor.setTargetPosition(1/*TODO:put target position number);
            while (elevatorMotor.getCurrentPosition() != /*TODO:put target posotion number){
                while (elevatorMotor.getCurrentPosition() < /*TODO: put target postition) {
                    elevatorMotor.setPower(1)/seudoPID;//TODO:dont tell me this is getting to PID
                    seudoPID++;
                }
                while (elevatorMotor.getCurrentPosition() > /*TODO: put target position){
                    elevatorMotor.setPower(-1/seudoPID);
                    seudoPID++;
                }
                if(seudoPID > 10){
                    break;
                }
            }
            elevatorMotor.setPower(0);
        }
        */

        if (gamepad2.dpad_up) {
            elevatorPower = 0.75;
        }
        else if (gamepad2.dpad_down) {
            elevatorPower = -0.75;
        }
        else if (gamepad2.dpad_left) {
            elevatorPower = -0.25;
        }
        else if (gamepad2.dpad_right) {
            elevatorPower = 0.25;
        }
        else {
            elevatorPower.setZeroPowerBehavior(DCMotorEX.ZeroPowerBehavior.BRAKE);
        }





        leftFrontPower    = forwardBackward + turning + straifing;
        rightFrontPower   = forwardBackward - turning - straifing;
        leftBackPower     = forwardBackward + turning - straifing;
        rightBackPower    = forwardBackward - turning + straifing;
        double[] powVals = {abs(leftFrontPower), abs(rightFrontPower), abs(leftBackPower), abs(rightBackPower)};
        Arrays.sort(powVals);
        if ((abs(leftFrontPower)) > 1 || (abs(rightFrontPower)) > 1 || (abs(leftBackPower) > 1) || (abs(rightBackPower) > 1))    {
             double maxPower = powVals[3];
             leftFrontPower    /= maxPower;
             rightFrontPower   /= maxPower;
             leftBackPower     /= maxPower;
             rightBackPower    /= maxPower;
              }


        if (gamepad2.a) {
            holderServo.setPosition(0.25/*TODO: put correct numbers*/);
        }
        else if (gamepad2.y) {
            holderServo.setPosition(0.75/*TODO: put correct number*/);
        }
        if (gamepad2.left_bumper){
            intakePower = 1;
        }
        else if (gamepad2.right_bumper){
            intakeMotor.setZeroPowerBehavior(DCMotorEX.ZeroPowerBehavior.BRAKE);
        }
        if(gamepad2.a){
            carouselPower = 0.65;
        else if(gamepad2.b){
            carouselMotor.setZeroPowerBehavior(DCMotorEX.ZeroPowerBehavior.BRAKE);
        }
        


        if (gamepad1.left_trigger > 0) {
            leftFrontPower = -0.3;
            leftBackPower = -0.3;
            rightFrontPower = 0.3;
            rightBackPower = 0.3;
        }
        else if (gamepad1.right_trigger > 0) {
            leftFrontPower = 0.3;
            leftBackPower = 0.3;
            rightFrontPower = -0.3;
            rightBackPower = -0.3;
        }
        // Change da world. my final nessage. goodby e
        // Send calculated power to wheels
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftBackWheel.setPower(leftBackPower);
        rightBackWheel.setPower(rightBackPower);
//        intakeMotor.setPower(intakePower);
        carouselMotor.setPower(carouselPower);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
                            //TODO: fix the following line to work with mecanum done?
        telemetry.addData("Motors", "left_front (%.2f), right_front (%.2f)", "left_back (%.2f)", "right_back (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

        /*
        gamepad1.dpad_direction
         *
         */

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
// change to the set power BRAKE thing??
    public void stop() {
        leftFrontWheel.setPower(0);
        rightFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightBackWheel.setPower(0);
        elevatorMotor.setPower(0);
        intakeMotor.setPower(0);
        carouselMotor.setPower(0);
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
