package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Robot: Ummdrive", group="drive")
public class RobotUmmdrive_Iterative extends OpMode {

    public DcMotor motor0   = null;
    public DcMotor  motor1  = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    @Override
    public void init() {

        // Define and Initialize Motors
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
    }




        @Override
        public void loop() {






            //fwrd = (1:f),(2:f),(3:f),(4:f)
            //leftspin = (1:b),(2:f),(3:b),(4:f)
            //rightspin = (1:f),(2:b),(3:f),(4:b)
            //back = (1:b),(2:b),(3:b),(4:b)
            //left = (1:b),(2:f),(3:f),(4:b)
            //right = (1:f),(2:b),(3:b),(4:f)



            if(gamepad1.right_stick_x == 1){

                    motor0.setPower(0.7);
                    motor1.setPower(-0.7);
                    motor2.setPower(-0.7);
                    motor3.setPower(-0.7);
            } //x = left/right

            if(gamepad1.right_stick_x == -1){

                    motor0.setPower(0.7);
                    motor1.setPower(-0.7);
                    motor2.setPower(-0.7);
                    motor3.setPower(-0.7);
            }

            if(gamepad1.right_stick_y == -1){

                    motor0.setPower(-0.7);
                    motor1.setPower(-0.7);
                    motor2.setPower(0.7);
                    motor3.setPower(-0.7);
            }

            if(gamepad1.right_stick_y == 1) {

                motor0.setPower(0.7);
                motor1.setPower(0.7);
                motor2.setPower(-0.7);
                motor3.setPower(0.7);

            } // y = forward/backwords

            if(gamepad1.right_stick_y == 0 && gamepad1.right_stick_x == 0){

                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
            }



        }
            double left;
            double right;
            @Override
       public void stop() {

            }
}




