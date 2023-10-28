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

    float RTtrigger;
    float LTtrigger;
    float RTtrigger2;
    @Override
    public void init() {

        // Define and Initialize Motors
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo3 = hardwareMap.get(Servo.class, "Servo2");
        RTtrigger = gamepad1.right_trigger;
        LTtrigger = gamepad1.left_trigger;
        RTtrigger2 = gamepad2.right_trigger;
    }




        @Override
        public void loop() {

            RTtrigger = gamepad1.right_trigger;
            LTtrigger = gamepad1.left_trigger;
            telemetry.addData("Trigger > ", RTtrigger);
            if (RTtrigger > 0.0) {
                Servo1.setPosition(0.5);
            } else {
                Servo1.setPosition(0.0);
            }
                telemetry.addData("Trigger > ", LTtrigger);
                if (LTtrigger > 0.0) {
                    Servo2.setPosition(0.5);
                } else {
                    Servo2.setPosition(0.0);
                }
                telemetry.addData("Trigger > ", RTtrigger2);
                if (RTtrigger2 > 0.0) {
                    Servo3.setPosition(0.5);
                } else {
                    Servo3.setPosition(0.0);
            }

        double x = gamepad1.right_stick_x;
        double y = -gamepad1.right_stick_y;
        double rx = gamepad1.left_stick_x;
        double sped = 0.7;
        boolean self_hang = gamepad1.y;
        boolean self_destruct = gamepad1.b;
          double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            motor0.setPower((-y - x - rx) / denominator * sped);
            motor1.setPower((y - x - rx) / denominator * sped);
            motor2.setPower((-y + x - rx) / denominator * sped);
            motor3.setPower((y + x - rx) / denominator * sped);
            if (self_hang) {
                motor4.setPower(0.4);

            }else {
                motor4.setPower(0);
            }
            if (self_destruct) {
                motor4.setPower(-0.2);
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
//            if(gamepad1.right_stick_y == 1) {
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

        }
            double left;
            double right;
            @Override
       public void stop() {

            }
}




