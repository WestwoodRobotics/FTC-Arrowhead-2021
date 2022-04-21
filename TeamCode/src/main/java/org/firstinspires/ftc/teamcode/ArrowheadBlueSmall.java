package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "arrowheadBlueSmall", group = "test")
public class ArrowheadBlueSmall extends LinearOpMode {
    SampleMecanumDrive drive;
    DcMotorEx carousel;

    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        carousel = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        waitForStart();
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(9.4)
                .strafeLeft(11*2.5)
                .build();
        drive.followTrajectorySequence(seq1);
        carousel.setPower(-0.2);
        sleep(5000);
        carousel.setPower(0);
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(8*2.5)
                .back(15)
                .turn(Math.toRadians(115))
                .forward(17)
                .strafeLeft(10)
                .build();
        drive.followTrajectorySequence(seq2);
        while(opModeIsActive()){
        }
    }
}