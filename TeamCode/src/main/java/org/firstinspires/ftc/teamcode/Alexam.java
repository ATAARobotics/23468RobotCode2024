package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Robot: poo", group="drive")
public class Alexam extends OpMode {

    public DcMotor ship  = null;
    public DcMotor  adam  = null;
    public DcMotor alexis  = null;
    public Servo temu;
    boolean RTbumper;
    IMU imu = null;

// ADAM + ALEXIS = ALEXAM

    @Override
    public void init() {

        ship = hardwareMap.get(DcMotor.class, "br");
        adam = hardwareMap.get(DcMotor.class, "bl");
//        alexis = hardwareMap.get(DcMotor.class, "fr");
        temu = hardwareMap.get(Servo.class, "bucket servo");
        imu = hardwareMap.get(IMU.class, "imu");
    }



        @Override
        public void loop() {

            double spinny = -gamepad1.right_stick_x;
            double rainbow = gamepad1.left_stick_y;
            RTbumper = gamepad1.right_bumper;
            double sped = 0.4;
            float fast = gamepad1.right_trigger;

//          Bucket Closer and Opener servo
            if (RTbumper) {
                temu.setPosition(-0.1);
            } else {
                temu.setPosition(0.4);
            }

//            If stick is less than 1 degrees, then nothing happens on gamepad1.right_stick_x and gamepad1.right_stick_y
            if (Math.abs (gamepad1.right_stick_x) < 0.05 && (Math.abs (gamepad1.left_stick_y) < 0.05)) {

                ship.setPower((-rainbow + spinny) * 0);
                adam.setPower((rainbow + spinny) * 0);

            } else {
                if (fast == 1) {
//                if press down on right trigger, go fast
                    ship.setPower(-rainbow + spinny);
                    adam.setPower(rainbow + spinny);
                } else {
                    ship.setPower((-rainbow + spinny) * sped);
                    adam.setPower((rainbow + spinny) * sped);
                }
            }



}}




