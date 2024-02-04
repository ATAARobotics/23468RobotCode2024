package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Robot: Ummdrive", group="drive")
public class RobotUmmdrive_Iterative extends OpMode {

    public DcMotor motor0   = null;
    public DcMotor  motor1  = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;
    public Servo Servo1;
    public Servo Servo2;
    public Servo frontdoor;//FrontDoor
    public Servo Servo4;
    public Servo Servodrone;
    public Servo ServoArm;
    public Servo ServoClaw;

    float RTtrigger;
    float LTtrigger;
    float LTtrigger2;
    float RTtrigger2;
    boolean Abutton;
    boolean Xbutton;
    boolean RTbumper;
    boolean LTbumper;


    double daveprev = 0;
    double davecur = 0;
    double daveyes = 0;
    double zerodave = 0;
    double jeffprev = 0;
    double jeffcur = 0;
    double jeffyes = 0;
    double zerojeff = 0;

    public DcMotor Dave = null;
    public DcMotor Jeff = null;
    IMU imu = null;
    double rotcur = 0;
    @Override
    public void init() {

        // Define and Initialize Motors
        motor0 = hardwareMap.get(DcMotor.class, "br");
        motor1 = hardwareMap.get(DcMotor.class, "bl");
        motor2 = hardwareMap.get(DcMotor.class, "fr");
        motor3 = hardwareMap.get(DcMotor.class, "fl");
        motor4 = hardwareMap.get(DcMotor.class, "Hang Motor");
        Servo1 = hardwareMap.get(Servo.class, "Hang Release"); //arm release
        Servo2 = hardwareMap.get(Servo.class, "Back Gate"); //back door
        frontdoor = hardwareMap.get(Servo.class, "Front Gate"); //front door
        Servodrone = hardwareMap.get(Servo.class, "Plane Launch");
        imu = hardwareMap.get(IMU.class, "imu");
        ServoArm = hardwareMap.get(Servo.class, "Left Arm");
        ServoClaw = hardwareMap.get(Servo.class, "Left Claw");

        Dave = hardwareMap.get(DcMotor.class, "Dave");//forward back
        Jeff = hardwareMap.get(DcMotor.class, "Jeff");//left right
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw();

        RTtrigger = gamepad1.right_trigger;
        LTtrigger = gamepad1.left_trigger;
        LTtrigger2 = gamepad2.left_trigger;
        RTtrigger2 = gamepad2.right_trigger;
        Abutton = gamepad2.a;
        Xbutton = gamepad2.x;

        Servo1.setPosition(0);
        Servo2.setPosition(0.3);
        frontdoor.setPosition(0);
        Servodrone.setPosition(0);
        ServoArm.setPosition(0.05);
        ServoClaw.setPosition(1);



    }




        @Override
        public void loop() {
            davecur = Dave.getCurrentPosition() - zerodave;
            jeffcur = Jeff.getCurrentPosition() - zerojeff;
            rotcur = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


            telemetry.addData("Jeff > ", jeffcur);
            telemetry.addData("Dave > ", davecur);


            RTtrigger2 = gamepad2.right_trigger;
            LTtrigger2 = gamepad2.left_trigger;
            Abutton = gamepad2.a;
            Xbutton = gamepad2.x;
            RTbumper = gamepad2.right_bumper;
            LTbumper = gamepad2.left_bumper;
            boolean self_hang = gamepad2.y;
            boolean self_destruct = gamepad2.b;

            telemetry.addData("Abutton > ", Abutton);
            if (Abutton) {
                Servo1.setPosition(0.75);
            } else {
                Servo1.setPosition(0);
            }
            telemetry.addData("Xbutton > ", Xbutton);
            if (Xbutton) {
                Servodrone.setPosition(1);
            } else {
                Servodrone.setPosition(0);
            }
            telemetry.addData("LTtrigger > ", LTtrigger2);
            if (LTtrigger2 > 0.0) {
                Servo2.setPosition(0);
            } else {
                Servo2.setPosition(0.3);
            }
            telemetry.addData("RTrigger > ",RTtrigger2);
            if (RTtrigger2 > 0.0) {
                frontdoor.setPosition(0.3);
            } else {
                frontdoor.setPosition(0.0);
            }
            telemetry.addData("RTbumper > ", RTbumper);
            if (LTbumper) {
               // Servo2.setPosition(0);
            } else {
                //Servo2.setPosition(0.3);
            }
            telemetry.addData("LTbumper > ",LTbumper);
            if (RTbumper) {
               // frontdoor.setPosition(0.3);
            } else {
               // frontdoor.setPosition(0.0);
            }

            double x = -gamepad1.right_stick_x;
            double y = gamepad1.right_stick_y;
            double rx = gamepad1.left_stick_x;
            double sped = 0.7;
            double rxSped = 0.5;



            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            motor0.setPower((-y - x - rx * rxSped) / denominator * sped);
            motor1.setPower((y - x - rx * rxSped) / denominator * sped);
            motor2.setPower((-y + x - rx * rxSped) / denominator * sped);
            motor3.setPower((y + x - rx * rxSped) / denominator * sped);



            /*
            double x_rot = x * Math.cos(rotcur*3.14/180) - y * Math.sin(rotcur*3.14/180);
            double y_rot = x * Math.sin(rotcur*3.14/180) + y * Math.cos(rotcur*3.14/180);

            double denominator = Math.max(Math.abs(y_rot) + Math.abs(x_rot) + Math.abs(rx), 1);
            motor0.setPower((-y_rot + x_rot - rx) / denominator * sped);
            motor1.setPower((y_rot + x_rot - rx) / denominator * sped);
            motor2.setPower((-y_rot - x_rot - rx) / denominator * sped);
            motor3.setPower((y_rot - x_rot - rx) / denominator * sped);

             */


            if (self_hang) {
                motor4.setPower(1);
            }else {
                motor4.setPower(0);
            }
            if (self_destruct) {
                motor4.setPower(-.8);
            }else {
                motor4.setPower(0);
            }

            //fwrd = (1:f),(2:f),(3:f),(4:f)
            //leftspin = (1:b),(2:f),(3:b),(4:f)
            //rightspin = (1:f),(2:b),(3:f),(4:b)
            //back = (1:b),(2:b),(3:b),(4:b)
            //left = (1:b),(2:f),(3:f),(4:b)
            //right = (1:f),(2:b),(3:b),(4:f)



//            if(gamepad1.right_stick_x == 1){
//
//                    motor0.setPower(0.7);
//                    motor1.setPower(-0.7);
//                    motor2.setPower(-0.7);
//                    motor3.setPower(-0.7);
//            } //x = left/right
//
//            if(gamepad1.right_stick_x == -1){
//
//                    motor0.setPower(0.7);
//                    motor1.setPower(-0.7);
//                    motor2.setPower(-0.7);
//                    motor3.setPower(-0.7);
//            }
//
//            if(gamepad1.right_stick_y == -1){
//
//                    motor0.setPower(-0.7);
//                    motor1.setPower(-0.7);
//                    motor2.setPower(0.7);
//                    motor3.setPower(-0.7);
//            }
//
//            if(gamepad1.right_stick   _y == 1) {
//
//                motor0.setPower(0.7);
//                motor1.setPower(0.7);
//                motor2.setPower(-0.7);
//                motor3.setPower(0.7);
//
//            } // y = forward/backwords
//
//            if(gamepad1.right_stick_y == 0 && gamepad1.right_stick_x == 0){
//
//                    motor0.setPower(0);
//                    motor1.setPower(0);
//                    motor2.setPower(0);
//                    motor3.setPower(0);
//            }
//
//

            daveyes = davecur - daveprev;
            daveprev = davecur;

            jeffyes = jeffcur - jeffprev;
            jeffprev = jeffcur;

        }
            double left;
            double right;
            @Override
       public void stop() {

            }
}




