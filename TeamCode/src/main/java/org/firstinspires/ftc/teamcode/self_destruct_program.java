package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.LinkedTransferQueue;

@TeleOp(name="Robot: bob", group="drive")
public class self_destruct_program extends OpMode {

    public DcMotor motor0   = null;
    public DcMotor  motor1  = null;
    public DcMotor motorPickup = null;
    public DcMotor Dave = null;
    public DcMotor Jeff = null;
    public Servo  servo0 = null;
    public Servo  servo1 = null;
    IMU imu = null;
    float RTtrigger;
    float Ltrigger;

    double rotcur = 0;
    @Override
    public void init() {

        // Define and Initialize Motors
        motor0 = hardwareMap.get(DcMotor.class, "br");
        motor1 = hardwareMap.get(DcMotor.class, "bl");
        motorPickup = hardwareMap.get(DcMotor.class, "pickup");
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        imu = hardwareMap.get(IMU.class, "imu");

        servo0.setPosition(0.925);
        servo1.setPosition(0);
    }
    @Override
        public void loop() {
            double y = -gamepad1.right_stick_y;
            double b = gamepad1.left_stick_y;
            double speed = 0.75;

            motor0.setPower(speed * b);
            motor1.setPower(speed * y);

            RTtrigger = gamepad1.right_trigger;

            if (RTtrigger > 0.0) {
                servo0.setPosition(0.8);
                servo1.setPosition(0.175);
            } else {
                servo0.setPosition(0.925);
                servo1.setPosition(0);


            }

            Ltrigger = gamepad1.left_trigger;

            if (Ltrigger > 0.0) {
                motorPickup.setPower(-0.3);
            } else {
                motorPickup.setPower( 0.3);
            }



        }
            double left;
            double right;
            @Override
       public void stop() {

            }
}



