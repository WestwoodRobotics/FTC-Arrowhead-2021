package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "ArrowheadRed", group = "test")
public class ArrowheadAutonTest extends LinearOpMode {
   SampleMecanumDrive drive;
   DcMotorEx carousel;

    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        carousel = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        waitForStart();
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(6*2.5)
                .forward(20)
                .build();
        drive.followTrajectorySequence(seq1);
        carousel.setPower(0.2);
        sleep(5000);
        carousel.setPower(0);
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(7.75*2.5)
                .back(125)
                .build();
        drive.followTrajectorySequence(seq2);
        while(opModeIsActive()){
        }
    }
}
