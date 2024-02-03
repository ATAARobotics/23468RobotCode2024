package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "CheckFrontCameraStreamRed", group = "Concept")
public class CheckFrontCameraStreamRed extends LinearOpMode {

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
