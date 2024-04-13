package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Robot: Ummdrive", group="drive")
public class self_destruct_program extends OpMode {

    public DcMotor motor0   = null;
    public DcMotor  motor1  = null;





    public DcMotor Dave = null;
    public DcMotor Jeff = null;
    IMU imu = null;
    double rotcur = 0;
    @Override
    public void init() {

        // Define and Initialize Motors
        motor0 = hardwareMap.get(DcMotor.class, "br");
        motor1 = hardwareMap.get(DcMotor.class, "bl");

        imu = hardwareMap.get(IMU.class, "imu");





    }

    @Override
        public void loop() {


            double y = gamepad1.right_stick_y;

            double b = gamepad1.left_stick_y;



            motor0.setPower(b);
            motor1.setPower(y);






        }
            double left;
            double right;
            @Override
       public void stop() {

            }
}




