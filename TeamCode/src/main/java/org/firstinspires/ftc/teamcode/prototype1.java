package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: motor0", group = "Concept")
public class prototype1 extends LinearOpMode {
    DcMotor motor0;
    DcMotor motor1;
float RTtrigger;
float LTtrigger;
    @Override
    public void runOpMode() throws InterruptedException {

        RTtrigger = gamepad1.right_trigger;
        LTtrigger = gamepad1.left_trigger;
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");

        waitForStart();

        while (opModeIsActive()) {
            RTtrigger = gamepad1.right_trigger;
            LTtrigger = gamepad1.left_trigger;
            telemetry.addData("Trigger > ", RTtrigger);
            if (RTtrigger   == 1) {

            }

        }
    }
}