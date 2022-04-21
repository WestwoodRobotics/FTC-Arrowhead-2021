package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "testTankAutonBlue", group = "test")
public class TankAuton extends LinearOpMode {
    Servo leftCarousel;
    Servo rightCarousel;
    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {
            new CustomMotor("leftFront"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightFront"),              //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("leftBack"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightBack"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("carousel"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rotateArm"),                //new PIDCoefficients(15, 0, 1))
            new CustomMotor("intake")
    };

    @Override
    public void runOpMode(){
        //Motor Initialization
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        leftCarousel = hardwareMap.get(Servo.class, "leftCarousel");
        rightCarousel = hardwareMap.get(Servo.class, "rightCarousel");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motors[0].motor = hardwareMap.get(DcMotorEx.class,"left Front");
        motors[1].motor = hardwareMap.get(DcMotorEx.class,"right Front");
        motors[2].motor = hardwareMap.get(DcMotorEx.class,"left Back");
        motors[3].motor = hardwareMap.get(DcMotorEx.class,"right Back");
        motors[4].motor = hardwareMap.get(DcMotorEx.class, "rotating Arm");
        motors[5].motor = hardwareMap.get(DcMotorEx.class, "intake");



        //Motor Zero Power Behavior
        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[4].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[5].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Motor PID Coefficients
        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[4].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[5].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);


        waitForStart();
        if(isStopRequested()){
            return;
        }
        Pose2d startPos = new Pose2d(0,0,0);
        drive.setPoseEstimate(startPos);
        leftCarousel.setPosition(1);
        rightCarousel.setPosition(1);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(11)
                .turn(Math.toRadians(220))
                .back(28)
                .waitSeconds(2)
                .turn(Math.toRadians(-300))
                .forward(35)
                .turn(Math.toRadians(120))
                .forward(15)
                .build();
        telemetry.speak("Auton started");
        drive.followTrajectorySequence(traj1);

        sleep(5000);
        leftCarousel.setPosition(0.5);
        rightCarousel.setPosition(0.5);
        runtime.reset();
        while(opModeIsActive()){

        }




    }
}
