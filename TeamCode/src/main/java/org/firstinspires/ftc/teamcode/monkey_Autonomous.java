package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Concept: Servo1", group = "Concept")
public class monkey_Autonomous extends LinearOpMode {
    Servo Servo0;
    DcMotor motor0;
    DcMotor motor1;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor Dave = null;
    public DcMotor Jeff = null;
    boolean monkey;

    VisionPortal portal1;
    AprilTagProcessor aprilTag;
    private TfodProcessor tfod;



    double sped = 0.7;
    double daveprev = 0;
    double davecur = 0;
    double daveyes = 0;
    double zerodave = 0;
    double jeffprev = 0;
    double jeffcur = 0;
    double jeffyes = 0;
    double zerojeff = 0;
    boolean redmonke;
    boolean listlength;


    int state=0;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo0 = hardwareMap.get(Servo.class, "Servo3");
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        Dave = hardwareMap.get(DcMotor.class, "Dave");
        Jeff = hardwareMap.get(DcMotor.class, "Jeff");

        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName("model_20231028_113246.tflite")
                //.setModelFileName()
                .setModelLabels(new String[] {"redmonke"})
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();
        portal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,  "Webbed Monkey Cameras"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tfod)
                .build();
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        zerojeff = Jeff.getCurrentPosition();
        zerodave = Dave.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {
            davecur = Dave.getCurrentPosition() - zerodave;
            jeffcur = Jeff.getCurrentPosition() - zerojeff;

            if (state==0) {
                redmonke = tfod.getRecognitions().size() > 0;
                if (redmonke) {
                    state = 1;
                }else {
                    state = 3;
                    motor0.setPower(1);
                    motor1.setPower(1);
                    motor2.setPower(1);
                    motor3.setPower(1) ;
                }

            }else if (state == 1) {
                motor0.setPower(1);
                motor1.setPower(-1);
                motor2.setPower(1);
                motor3.setPower(-1) ;

                if (davecur >= 8000) {
                    state = 2;
                    zerodave = Dave.getCurrentPosition();
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }

            }else if (state == 2){
                Servo0.setPosition(0.0);
                // dump pixel
            }else if (state == 3){
                motor0.setPower(-1);
                motor1.setPower(-1);
                motor2.setPower(1);
                motor3.setPower(1);
               if (davecur >= 8000) {
                   zerodave = Dave.getCurrentPosition();
                   motor0.setPower(0);
                   motor1.setPower(0);
                   motor2.setPower(0);
                   motor3.setPower(0);
               state = 4;
               }

            }else if (state == 4){
                redmonke = tfod.getRecognitions().size() > 0;
                if (redmonke) {
                    state = 5;
                }
            }else if (state == 5){
                motor0.setPower(1);
                motor1.setPower(-1);
                motor2.setPower(1);
                motor3.setPower(-1);
               if (davecur >= 8000) {
                   zerodave = Dave.getCurrentPosition();
                   motor0.setPower(0);
                   motor1.setPower(0);
                   motor2.setPower(0);
                   motor3.setPower(0);
                   state = 2;
               }

            }else if (state == 6){

                // if no monkey on line2
            }else if (state == 7){
                motor0.setPower(1);
                motor1.setPower(1);
                motor2.setPower(1);
                motor3.setPower(1) ;
                // turn 108' right

            }else if (state == 8){

                // drive to monkey/line3
            }


            daveyes = davecur - daveprev;
            daveprev = davecur;

            jeffyes = jeffcur - jeffprev;
            jeffprev = jeffcur;




            telemetry.addData("Dave's change", daveyes);

            telemetry.addData("Dave's current data", davecur);



            telemetry.update();
        }
    }
}