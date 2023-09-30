package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: Servo0", group = "Concept")
public class prototype  extends LinearOpMode {
    Servo Servo0;
    DcMotor motor0;
    DcMotor motor1;
float RTtrigger;
float LTtrigger;
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
            

            telemetry.update();
        }
    }
}