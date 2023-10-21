package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: Servo0", group = "Concept")
public class  prototype  extends LinearOpMode {
    Servo Servo0;
    DcMotor motor0;
    DcMotor motor1;
float RTtrigger;
float LTtrigger;

int state=0;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo0 = hardwareMap.get(Servo.class, "Servo0");
        RTtrigger = gamepad1.right_trigger;
        LTtrigger = gamepad1.left_trigger;
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");

        waitForStart();

        while (opModeIsActive()) {
            RTtrigger = gamepad1.right_trigger;
            LTtrigger = gamepad1.left_trigger;
            telemetry.addData("Trigger > ", RTtrigger);
            if (RTtrigger > 0.0) {
                Servo0.setPosition(0.5);
            } else {
                Servo0.setPosition(0.0);
            }


            if (state==0) {//check center 1

            }else if (state == 1) {
                // drive to 1
            }else if (state == 2){
                // dump pixel
            }else if (state ==3){
                // if no monkey on line1
            }else if (state == 4){
                // drive to center
            }else if (state == 5){
                // turn 90' right
            }else if (state == 6){
                // if no monkey on line2
            }else if (state == 7){
                // turn 108' right
            }else if (state == 8){
                // drive to monkey
            }else if (state == 2){
                // dump pixel
            }

            

            telemetry.update();
        }
    }
}