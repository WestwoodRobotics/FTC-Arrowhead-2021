package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.BasicPermission;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "camera", group = "test")
public class CameraDetectionTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY ="AfXmAXv/////AAABmZaSDB4kxkuDoqNpk5yUzVAXaAR/JH4uUmFpVKxSTpWaQI6MCH8uYxF829niaq3Tx/xULuqyU3KUxo3D8nUl07LCkhiIfwMUCUkUqsCqpiCU9hDItwnZ0rE9UZe0pyf10lIWBuxZZQ/G/GEq51H7lX1TpJrCwSKQNkiKxQFhygLVRcOq5vcIMH3xHJsFgUJVHridW06NEtO8pEY/muowgupfgK97TFT2w1jkx+QI97f8S3Q6DBXVuhNy5mLTc3b3puh6p1qwU+sFrI9Azha9SdwWBxvzOEeNV3nEQ2bWqyBz9+fnREa5WxDSGNKekGNVHcJ5LPzyIM0OKWlmRvEpDxGvg0ABgiI0Hnhj/crR/f65";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private double firstLine = 1280/3.0;
    private double secondLine = 2*1280/3.0;
    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.3f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    @Override
    public void runOpMode(){
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        int level = 0;
        waitForStart();
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if(updatedRecognitions.size() > 0){
                        Recognition last = updatedRecognitions.get(updatedRecognitions.size()-1);
                        if(last.getLabel().equals("Duck")){
                            double midX = (double)((last.getRight() - last.getLeft())/2+last.getLeft());
                            if(midX <= firstLine){
                                level = 1;
                            } else if(midX <= secondLine){
                                level = 2;
                            } else{
                                level = 3;
                            }
                        }
                    }
                    telemetry.addData("level detected", level);
                    telemetry.addData("# Object Detected", updatedRecognitions.size());


                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                    telemetry.update();

                }
            }
        }
    }
}
