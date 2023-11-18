package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Robot: Ummdrive", group="drive")
public class RobotUmmdrive_Iterative extends OpMode {

    public DcMotor motor0   = null;
    public DcMotor  motor1  = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;
    public Servo Servo1;
    public Servo Servo2;
    public Servo Servo3;
    public Servo Servo4;

    float RTtrigger;
    float LTtrigger;
    float LTtrigger2;
    float RTtrigger2;
    boolean Abutton;
    boolean Xbutton;

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
    @Override
    public void init() {

        // Define and Initialize Motors
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        Servo1 = hardwareMap.get(Servo.class, "Servo1"); //arm release
        Servo2 = hardwareMap.get(Servo.class, "Servo2"); //back door
        Servo3 = hardwareMap.get(Servo.class, "Servo3"); //front door
        Servo4 = hardwareMap.get(Servo.class, "Servo4");

        Dave = hardwareMap.get(DcMotor.class, "Dave");
        Jeff = hardwareMap.get(DcMotor.class, "Jeff");

        RTtrigger = gamepad1.right_trigger;
        LTtrigger = gamepad1.left_trigger;
        LTtrigger2 = gamepad2.left_trigger;
        RTtrigger2 = gamepad2.right_trigger;
        Abutton = gamepad2.a;
        Xbutton = gamepad2.x;

        Servo1.setPosition(0);
        Servo2.setPosition(0.3);
        Servo3.setPosition(0);
        Servo4.setPosition(1);


    }




        @Override
        public void loop() {
            davecur = Dave.getCurrentPosition() - zerodave;
            jeffcur = Jeff.getCurrentPosition() - zerojeff;


            telemetry.addData("Jeff > ", jeffcur);
            telemetry.addData("Dave > ", davecur);


            RTtrigger2 = gamepad2.right_trigger;
            LTtrigger2 = gamepad2.left_trigger;
            Abutton = gamepad2.a;
            Xbutton = gamepad2.x;

            telemetry.addData("Abutton > ", Abutton);
            if (Abutton) {
                Servo1.setPosition(0.75);
            } else {
                Servo1.setPosition(0);
            }
            telemetry.addData("Xbutton > ", Xbutton);
            if (Xbutton) {
                Servo4.setPosition(0);
            } else {
                Servo4.setPosition(1);
            }
            telemetry.addData("RTrigger > ", LTtrigger2);
            if (LTtrigger2 > 0.0) {
                Servo2.setPosition(0);
            } else {
                Servo2.setPosition(0.3);
            }
            telemetry.addData("RTrigger > ",RTtrigger2);
            if (RTtrigger2 > 0.0) {
                Servo3.setPosition(0.3);
            } else {
                Servo3.setPosition(0.0);
            }
            double x = gamepad1.right_stick_x;
            double y = -gamepad1.right_stick_y;
            double rx = gamepad1.left_stick_x;
            double sped = 0.7;
            double rxSped = 0.5;
            boolean self_hang = gamepad2.y;
            boolean self_destruct = gamepad2.b;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motor0.setPower((-y - x - rx * rxSped) / denominator * sped);
            motor1.setPower((y - x - rx * rxSped) / denominator * sped);
            motor2.setPower((-y + x - rx * rxSped) / denominator * sped);
            motor3.setPower((y + x - rx * rxSped) / denominator * sped);

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




