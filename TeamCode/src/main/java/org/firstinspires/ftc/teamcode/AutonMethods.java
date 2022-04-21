package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AutonMethods {
    public static boolean Move(CustomMotor[] motors, double y, double r, double currentTime, double start, double time,double pow){
        boolean stop = false;
        double[] velocity;
        if(currentTime-start>=time){
            velocity = new double[]{
                    0,
                    0,
                    0,
                    0
            };
            stop = true;
        } else{
            velocity = new double[]{
                    (y + r),    //Front Left
                    (y - r),    //Front Right
                    (y + r),    //Back Left
                    (y - r)     //Back Right
            };
        }
        for(int i = 0; i < 4; i++){
            motors[i].motor.setPower(pow*velocity[i]);
        }
        return stop;

    }
    public static boolean carousel(CustomMotor[] motors, double currentTime, double start, double time){
        boolean stop = false;
        if(currentTime-start>=time){
            motors[4].motor.setPower(0);
            stop = true;
        } else{
            motors[4].motor.setPower(-0.25);
        }
        return stop;
    }
    public static boolean carousel2(CustomMotor[] motors, double currentTime, double start, double time){
        boolean stop = false;
        if(currentTime-start>=time){
            motors[4].motor.setPower(0);
            stop = true;
        } else{
            motors[4].motor.setPower(0.25);
        }
        return stop;
    }

}
