package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: Servo0", group = "Concept")
public class prototype  extends LinearOpMode {

    Servo Servo0;
float RTtrigger;
    @Override
    public void runOpMode() throws InterruptedException {

        Servo0 = hardwareMap.get(Servo.class, "Servo0");
        RTtrigger = gamepad1.right_trigger;

        while (opModeIsActive()) {
            RTtrigger = gamepad1.right_trigger;
            if (RTtrigger > 0.0) {
                Servo0.setPosition(0.5);
            }
            Servo0.setPosition(0.0);
        }
    }
}