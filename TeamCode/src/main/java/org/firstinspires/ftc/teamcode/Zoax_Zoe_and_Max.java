package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import kotlin.math.UMathKt;

@TeleOp(name="Robot: poo", group="drive")
public class Zoax_Zoe_and_Max extends OpMode {

    public DcMotor ship  = null;
    public DcMotor  max  = null;
//    public DcMotor zoe  = null;
    IMU imu = null;



    @Override
    public void init() {

        ship = hardwareMap.get(DcMotor.class, "br");
        max = hardwareMap.get(DcMotor.class, "bl");
//        zoe = hardwareMap.get(DcMotor.class, "fr");
        imu = hardwareMap.get(IMU.class, "imu");
    }



        @Override
        public void loop() {

            double spinny = -gamepad1.right_stick_x;
            double rainbow = gamepad1.left_stick_y;
            double sped = 0.4;

            float fast = gamepad1.right_trigger;
//            If stick is less than 1 degrees, then nothing happens on gamepad1.right_stick_x and gamepad1.right_stick_y
            if (Math.abs (gamepad1.right_stick_x) < 0.05 && (Math.abs (gamepad1.left_stick_y) < 0.05)) {

                ship.setPower((-rainbow + spinny) * 0);
                max.setPower((rainbow + spinny) * 0);

            } else { if (fast > 1) {
//                if press down on right trigger, go fast
                ship.setPower(-rainbow + spinny);
                max.setPower(rainbow + spinny);

            } else {

                ship.setPower((-rainbow + spinny) * sped);
                max.setPower((rainbow + spinny) * sped);
            }
            }



}}




