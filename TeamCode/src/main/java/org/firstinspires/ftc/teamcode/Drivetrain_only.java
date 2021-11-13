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

package org.firstinspires.ftc.teamcode;
//done other than testing?

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEX;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")

public class DrivetrainOnly extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEX leftFrontWheel;
    private DcMotorEX rightFrontWheel;
    private DcMotorEX leftBackWheel;
    private DcMotorEX rightBackWheel;
    private DcMotorEX carouselMotor;
    private Servo holderServo;
    private DcMotorEX elevatorMotor;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontWheel  = hardwareMap.get(DcMotorEX.class, "leftFrontWheel");
        rightFrontWheel = hardwareMap.get(DcMotorEX.class, "rightFrontWheel");
        leftBackWheel = hardwareMap.get(DcMotorEX.class, "leftBackWheel");
        rightBackWheel = hardwareMap.get(DcMotorEX.class, "rightBackWheel");
        carouselMotor = hardwareMap.get(DcMotorEX.class, "carouselMotor");
        holderServo = hardwareMap.get(DcMotorEx.class, "holderServo");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");

        //TODO: add max and min values for servo
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontWheel.setDirection(DcMotorEX.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotorEX.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorEX.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotorEX.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorEX.Direction.FORWARD);
        holderServo.setDirection(Servo.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorEx.Direction.FORWARD);

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
        double carouselPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double forwardBackward = gamepad1.left_stick_y;
        double straifing =  -gamepad1.left_stick_x;
        double turning = gamepad1.right_stick_x;


        //double elevatorHeight = elevatorMotor.getCurrentPosition(); TODO: test position values

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


        if(gamepad2.left_bumper == True) {
            holderServo.setPosition(0.25);
        else if(gamepad2.right_bumper == True){
            holderServo.setPosition(0.0);
        
        
        
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
