package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "Autonomous-Red-Back", group = "Concept")
public class monkey_Autonomous_Red_Back extends LinearOpMode {
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



    double sped = 0.3;
    double daveprev = 0;
    double davecur = 0;
    double rotcur = 0;
    double daveyes = 0;
    double zerodave = 0;
    double jeffprev = 0;
    double jeffcur = 0;
    double jeffyes = 100;
    double[] jeffyes_buff = new double[100];
    double[] daveyes_buff = new double[100];
    double zerojeff = 0;
    boolean redmonke = false;
    boolean listlength;
    double pos_x;
    double pos_y;
    double rot = 0;
    double dave_distance_left;
    double prev_dave_distance_left;
    double dave_i = 0;
    double jeff_distance_left;
    double prev_jeff_distance_left;
    double jeff_i = 0;
    double rot_left;
    double rot_i = 0;
    double start_time;
    double loop_time = 1;
    double x_change = 1;
    double y_change = 1;
    IMU imu;



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
        imu = hardwareMap.get(IMU.class, "imu");

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
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw();

        Arrays.setAll(jeffyes_buff, i -> 100);
        Arrays.setAll(daveyes_buff, i -> 100);

        Servo0.setPosition(0.2);

        zerojeff = Jeff.getCurrentPosition();
        zerodave = Dave.getCurrentPosition();

        while (opModeInInit()) {
            davecur = Dave.getCurrentPosition() - zerodave;
            jeffcur = Jeff.getCurrentPosition() - zerojeff;
            rotcur = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("Jeff init > ", jeffcur);
            telemetry.addData("Dave init > ", davecur);

            telemetry.addData("Jeff rot > ", rotcur * 393);
            telemetry.addData("Dave rot > ", rotcur * 2376);



            telemetry.update();
        }

        waitForStart();


        while (opModeIsActive()) {
            start_time = getRuntime();
            telemetry.addData("State > ", state);

            y_change = prev_jeff_distance_left - jeff_distance_left;
            x_change = prev_dave_distance_left - dave_distance_left;

            rotcur = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            davecur = Dave.getCurrentPosition() - zerodave - rotcur * 3.14/180 * 2500;
            jeffcur = Jeff.getCurrentPosition() - zerojeff - rotcur * 3.14/180 * 400;

            telemetry.addData("Rot",rotcur*180/3.14);

            daveyes = davecur - daveprev; // change
            jeffyes = jeffcur - jeffprev;
            for (int i = 1; i < jeffyes_buff.length; i++) {
                jeffyes_buff[i] = jeffyes_buff[i-1];
            }
            jeffyes_buff[0] = jeffyes;
            for (int i = 1; i < daveyes_buff.length; i++) {
                daveyes_buff[i] = daveyes_buff[i-1];
            }
            daveyes_buff[0] = daveyes;


            daveyes *= 1; // offset due to build angle
            jeffyes *= 0.8;

            dave_distance_left = pos_x - davecur;
            dave_i += dave_distance_left * loop_time;
            jeff_distance_left = pos_y - jeffcur;
            jeff_i += jeff_distance_left * loop_time;
            rot_left = rot - rotcur;
            rot_i += rot_left * 3.14/180 * loop_time;

            double y = 0.0025*(dave_distance_left);// - 0.0025 * dave_i;
            double x = 0.0025*(jeff_distance_left);// - 0.0001 * jeff_i;
            double rx = -1.9 * rot_left * 3.14/180;// + 0.4 * rot_i;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motor0.setPower((-y + x - rx) / denominator * sped);
            motor1.setPower((y + x - rx) / denominator * sped);
            motor2.setPower((-y - x - rx) / denominator * sped);
            motor3.setPower((y - x - rx) / denominator * sped);

            telemetry.addData("Jeff > ", jeffcur);
            telemetry.addData("Dave > ", davecur);


            if (state==0) {
                //redmonke = tfod.getRecognitions().size() > 0;
                if (redmonke) {
                    //state = 1;
                }else {
                    state = 3;

                }

            }else if (state == 1) { //drive forward
                motor0.setPower(1);
                motor1.setPower(-1);
                motor2.setPower(1);
                motor3.setPower(-1) ;

                if (davecur >= 8000) { //stop and go to dump
                    state = 2;
                    zerodave = Dave.getCurrentPosition();
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                }

            }else if (state == 2){ // dump pixel
                Servo0.setPosition(0.0);
                state = 5; // park close
            }else if (state == 3){ // move to side
                pos_x = 0;
                pos_y = -3200;

                if (redmonke && Arrays.stream(jeffyes_buff).sum() == 0.0 && pos_y - 40 < jeffcur && jeffcur < pos_y + 40) {
                    zerojeff = Jeff.getCurrentPosition();
                    pos_y = 0;
                    //state = 4;
                } else if (Arrays.stream(jeffyes_buff).sum() == 0.0 && pos_y - 40 < jeffcur && jeffcur < pos_y + 40) {
                    zerojeff = Jeff.getCurrentPosition();
                    pos_y = 0;
                    state = 5;
                }
            }else if (state == 4){
                redmonke = tfod.getRecognitions().size() > 0;
                if (redmonke) { // yes monkey go to drive forward
                    state = 5;
                } else {
                    state = 6;
                }
            }else if (state == 5){ // dive to line
                pos_x = 9251;
                pos_y = 0;

                telemetry.addData("dave buff", Arrays.stream(daveyes_buff).sum());

                if (Arrays.stream(daveyes_buff).sum() == 0.0 && pos_x - 40 < davecur && davecur < pos_x + 40) {
                    zerodave = Dave.getCurrentPosition();
                    pos_x = 0;
                    state = 6;
                }
            }else if (state == 6){ // rotate
                pos_x = 0;
                pos_y = 0;
                rot = 90;
                if (rot - 0.5 < rotcur && rotcur < rot + 0.5) {
                    zerodave = Dave.getCurrentPosition();
                    zerojeff = Jeff.getCurrentPosition();
                    state = 7;
                }
            }else if (state == 7){ // forward drop
                pos_x = 100;
                pos_y = 0;

                if (Arrays.stream(daveyes_buff).sum() == 0.0 && pos_x - 40 < davecur && davecur < pos_x + 40) {
                    state = 8;
                    Servo0.setPosition(0.0);
                    pos_x = 0;
                    zerodave = Dave.getCurrentPosition();
                }


            }else if (state == 8){ // park
                pos_x = -8000;
                pos_y = 9751;
                rot = 0;
            }




            daveprev = davecur;
            jeffprev = jeffcur;
            prev_dave_distance_left = dave_distance_left;
            prev_jeff_distance_left = jeff_distance_left;

            telemetry.addData("Dave's change", daveyes);

            telemetry.update();
            loop_time = start_time - getRuntime();
        }
    }
}