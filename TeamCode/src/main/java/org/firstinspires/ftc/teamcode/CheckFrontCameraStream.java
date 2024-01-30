package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;



@Autonomous(name = "CheckFrontCameraStream", group = "Concept")
public class CheckFrontCameraStream extends LinearOpMode {

    public CraigCameraPipeline cameraPipeline = new CraigCameraPipeline(false); // -------------------------------------------------------------------CHANGE THIS TO DETECT RED/BLUE

    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Front Camera");
        OpenCvCamera frontCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        frontCamera.setPipeline( cameraPipeline );

        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode) {}
        });

        while (opModeInInit()) {

            telemetry.addData("isMonkeyVisible?", cameraPipeline.doISeeMonkey());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {


            telemetry.update();

        }

    }

}
