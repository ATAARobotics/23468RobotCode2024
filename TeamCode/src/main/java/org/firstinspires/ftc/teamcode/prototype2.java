package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name = "Concept: Servo0", group = "Concept")
public class prototype2 extends LinearOpMode {
    Servo Servo0;
    DcMotor motor0;
    DcMotor motor1;
float RTtrigger;
float LTtrigger;
boolean Abutton;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    double x = gamepad1.right_stick_x;
    double y = -gamepad1.right_stick_y;
    double rx = gamepad1.left_stick_x;
    public DcMotor Dave = null;
    boolean monkey = true;

    VisionPortal portal1;
    AprilTagProcessor aprilTag;
    private TfodProcessor tfod;



    double sped = 0.7;
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double daveprev = 0;
    double davecur = 0;
    double daveyes = 0;
    double zerodave = 0;
    boolean redmonke;
    boolean listlength;


    int state=0;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo0 = hardwareMap.get(Servo.class, "Servo0");
        RTtrigger = gamepad1.right_trigger;
        LTtrigger = gamepad1.left_trigger;
        Abutton = gamepad1.a;
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        Dave = hardwareMap.get(DcMotor.class, "Dave");
        redmonke = tfod.getRecognitions().size() > 0;

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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(tfod)
                .build();
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        waitForStart();

        while (opModeIsActive()) {
            davecur = Dave.getCurrentPosition() - zerodave;
            RTtrigger = gamepad1.right_trigger;
            LTtrigger = gamepad1.left_trigger;
            telemetry.addData("Trigger > ", RTtrigger);
            if (RTtrigger > 0.0) {
                Servo0.setPosition(0.5);
            } else {
                Servo0.setPosition(0.0);
            }


            if (state==0) {
                redmonke = tfod.getRecognitions().size() > 0;
                if (redmonke) {
                    state = 1;
                }else {
                    state = 3;
                    motor0.setPower();
                    motor1.setPower();
                    motor2.setPower();
                    motor3.setPower() ;
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
                if (RTtrigger > 0.0) {
                    Servo0.setPosition(0.5);
                } else {
                    Servo0.setPosition(0.0);
                }
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
            }else if (state == 2){
                if (RTtrigger > 0.0) {
                    Servo0.setPosition(0.5);
                } else {
                    Servo0.setPosition(0.0);
                }
                // dump pixel
            }


            daveyes = davecur - daveprev;
            daveprev = davecur;





            telemetry.addData("Dave's change", daveyes);

            telemetry.addData("Dave's current data", davecur);



            telemetry.update();
        }
    }
}