package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "test Tank", group = "TeleOp")
public class TankTeleop extends OpMode {
    double armTicks = 286.2*28.0;
    boolean carouselDirection = true;

    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {new CustomMotor("leftFront"),new CustomMotor("rightFront"),new CustomMotor("leftBack"),new CustomMotor("rightBack"),new CustomMotor("rotateArm"),new CustomMotor("intake")};
    Servo leftCarousel;
    Servo rightCarousel;
    boolean slowMode = false;
    double velocityMultiplier = 1;
    double tickTolerance = 90;
    double carouselPos = 0.5;


    @Override
    public void init() {
        leftCarousel = hardwareMap.get(Servo.class, "leftCarousel");
        rightCarousel = hardwareMap.get(Servo.class, "rightCarousel");

        //Motor Initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motors[0].motor = hardwareMap.get(DcMotorEx.class,"left Front");
        motors[1].motor = hardwareMap.get(DcMotorEx.class,"right Front");
        motors[2].motor = hardwareMap.get(DcMotorEx.class,"left Back");
        motors[3].motor = hardwareMap.get(DcMotorEx.class,"right Back");
        motors[4].motor = hardwareMap.get(DcMotorEx.class, "rotating Arm");
        motors[5].motor = hardwareMap.get(DcMotorEx.class, "intake");

        //Motor Direction
        motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[4].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[5].motor.setDirection(DcMotorEx.Direction.FORWARD);

        //Motor Zero Power Behavior
        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[5].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Motor PID Coefficients
        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[4].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[5].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
    }



    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        //Velocity Multiplier

        if(gamepad1.dpad_up && slowMode){
            slowMode = false;
            velocityMultiplier = 1;
            gamepad1.rumble(200);
        } else if(gamepad1.dpad_down && !slowMode){
            slowMode = true;
            velocityMultiplier = 0.2;
            gamepad1.rumble(200);
        }

        double y = -gamepad1.left_stick_y; //forward
        double r = gamepad1.right_stick_x; //turn

        //Individual Wheel Velocity
        double[] velocity = {
                (y + r),    //Front Left
                (y - r),    //Front Right
                (y + r),    //Back Left
                (y - r)     //Back Right
        };

        double highestValue = 0;
        for (double ix : velocity) {
            if (Math.abs(ix) > highestValue) {
                highestValue = Math.abs(ix);
            }
        }

        if (highestValue > 1) {
            for (double ix : velocity) {
                ix /= highestValue;
            }
        }

        //Velocity Multiplier, Motor RPM
        for (int i = 0; i < 4; i++) {
            motors[i].motor.setVelocity(velocity[i]*velocityMultiplier*5000);
        }

        //Carousel
        if(gamepad1.right_bumper){
            carouselDirection = !carouselDirection;
            leftCarousel.setPosition(1);
            rightCarousel.setPosition(1);
        } else if (gamepad1.left_bumper) {
            leftCarousel.setPosition(0);
            rightCarousel.setPosition(0);
        } else {
            leftCarousel.setPosition(0.5);
            rightCarousel.setPosition(0.5);
        }

        //Rotating Arm
        if(gamepad2.right_stick_y != 0){
            motors[4].motor.setPower(gamepad2.right_stick_y);
        } else {
            motors[4].motor.setPower(0);
        }

        //Intake
        if (gamepad2.right_trigger>0) {
            motors[5].motor.setVelocity(5000*gamepad2.right_trigger);
        } else if (gamepad2.left_trigger>0) {
            motors[5].motor.setVelocity(-1000*gamepad2.left_trigger);
        } else {
            motors[5].motor.setVelocity(0);
        }

        //Captions, Information Display
        telemetry.addData("Status",             "Run Time: " + runtime.toString());
        telemetry.addData("a: ",                       velocityMultiplier);
        telemetry.addData("FRONT LEFT Motor",   motors[0].motor.getVelocity() + "rps");
        telemetry.addData("FRONT RIGHT Motor",  motors[1].motor.getVelocity() + "rps");
        telemetry.addData("BACK LEFT Motor",    motors[2].motor.getVelocity() + "rps");
        telemetry.addData("BACK RIGHT Motor",   motors[3].motor.getVelocity() + "rps");
        telemetry.addData("Rotating Arm power",       motors[4].motor.getPower() + "rps");
        telemetry.addData("rotating arm position", motors[4].motor.getCurrentPosition());

        telemetry.update();
    }

}
