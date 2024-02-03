package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.Servo;

//DONE: Create a drive right a distance func
//      DONE: Replicate for when we are moving Left and right
//      DONE: LEFT RIGHT drift vs FB drift
//DONE: set deadzones to variables
//DONE: Red object detection
//DONE: Second Camera Setup
//DONE: QR Codes
//TODO: GO TO QR CODE
//TODO: Move at diagonal
//TODO: Give the board a slap



@Autonomous(name = "CraigDrive", group = "Concept")
public class CraigDrive extends LinearOpMode {
    public Motor motor0_br = null; //Back right
    public Motor motor1_bl = null; //Back left
    public Motor motor2_fr = null; //front right
    public Motor motor3_fl = null; //front left
    public Servo dropPixel = null;
    public Servo leftArm = null;
    public Servo leftClaw = null;

    public double targetPower = 0.0;
    public double currPower = 0.0;

    public double targetHeading = 0.0;
    public int targetDistance_fb = 0;
    public int targetDistance_lr = 0;

    public double targetHeadingLastItr = -1.0;
    public double targetDistance_fbLastItr = -1;
    public double targetDistance_lrLastItr = -1;

    public int frameCountInStandstill = 0;

    public CurrentMotionType currentMotionType = CurrentMotionType.ROTATE;

    public int motor0target = 0;
    public int motor1target = 0;
    public int motor2target = 0;
    public int motor3target = 0;

    double endOfWaitTime;
    boolean isWaiting = false;

    public DcMotor Dave_fb = null; //dave is forward/backwards, jeff is side to side
    public int zeroDave_fb = 0;

    public DcMotor Jeff_lr = null; //dave is forward/backwards, jeff is side to side
    public int zeroJeff_lr;

    public IMU imu;
    public RevHubOrientationOnRobot orientationOnRobot;

    public BlockOfActions placePixelInCentre = new BlockOfActions("placePixelInCentre");
    public BlockOfActions goToRightForCheck = new BlockOfActions("goToRightForCheck");
    public BlockOfActions placePixelOnRight = new BlockOfActions("placePixelOnRight");
    public BlockOfActions placePixelOnLeft = new BlockOfActions("placePixelOnLeft");

    public BlockOfActions testBlock = new BlockOfActions("testBlock");

    public StateFlowAbstract initialState = null;
    public StateFlowAbstract rightCheckForMonkey = null;

    public AutoAction currAction = null;
    public BlockOfActions currBlock = null;
    public StateFlowAbstract nextStateToLoad = null;

    public CraigCameraPipeline cameraPipeline = new CraigCameraPipeline(false); // -------------------------------------------------------------------CHANGE THIS TO DETECT RED/BLUE

    final double fx = 466.664;
    final double fy = 466.664;
    final double cx = 114.847;
    final double cy = 170.026;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public AprilTagDetection detectedTag;
    public int framesSinceLastTagSighting = 0;
    public boolean centeredOnTag = false;
    public boolean tooCloseToDetectSlideInTime = false;

    public Motor initializeMotor(String motorName){
        Motor m;
        m = new Motor(hardwareMap, motorName); //Use the ftclib hardware map
        m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); //set default state, BREAK or FLOAT (Float is default)
        m.setRunMode(Motor.RunMode.PositionControl); //Run mode, options are: RawPower, PositionControl, VelocityControl
        m.setPositionCoefficient(0.05); //magic numbers to tune each motor to make the motors all act the same
        m.setPositionTolerance(1.0);
        m.resetEncoder(); //Zeros the encoder can call this at any time
        return m;

    }

    public void executeCurrAction() {
        if (currAction == null) {
            return;
        }

        switch (currAction.actionType) {
            case "faceHeading":
                currentMotionType = CurrentMotionType.ROTATE;
                this.setHeading(currAction.amt);
                break;
            case "driveForwardADistance":
                currentMotionType = CurrentMotionType.FORWARD;
                this.driveForwardADistance((int) currAction.amt);
                break;
            case "driveRightADistance":
                currentMotionType = CurrentMotionType.RIGHT;
                this.driveRightADistance((int) currAction.amt);
                break;
            case "dropPixel":
                currentMotionType = CurrentMotionType.NO;
                this.dropPixel();
                break;
            case "wait":
                wait((int) currAction.amt);
                break;
            case "navigateToAprilTag":
                driveToAprilTag((int) currAction.amt);
                break;
            case "theBIGSlap":
                theBIGSlap();
                break;
        }
    }

    public boolean checkConditions() {
        if (nextStateToLoad.condition == null) {
            return true;
        }
        switch (nextStateToLoad.condition) {
            case "monkeyIsVisible":
                telemetry.addData("monkeyIsVisible", cameraPipeline.doISeeMonkey());
                return cameraPipeline.doISeeMonkey();
            case "secondConditionToSuppressWarning":
                return true;
            default:
                return false;
        }
    }

    public void nextBlockOfActionsIs() {
        if (nextStateToLoad == null) {
            return;
        }

        if (checkConditions()) {
            currBlock = nextStateToLoad.nextBlockIfTrue;
            nextStateToLoad = nextStateToLoad.nextStateIfTrue;
        } else {
            currBlock = nextStateToLoad.nextBlockIfFalse;
            nextStateToLoad = nextStateToLoad.nextStateIfFalse;
        }
    }

    public void actionCompleteCommanderReadyForNextCommand() {
        motor0_br.resetEncoder();
        motor1_bl.resetEncoder();
        motor2_fr.resetEncoder();
        motor3_fl.resetEncoder();
        resetMotors();
        currPower = 0.0;
        targetPower = 0.0;
        zeroDave_fb = Dave_fb.getCurrentPosition();
        zeroJeff_lr = Jeff_lr.getCurrentPosition();
        targetDistance_fb = 0;
        targetDistance_lr = 0;
        //maintain target heading

        if (currBlock != null)  {
            currAction = currBlock.autoActionQueue.poll();
        }

        if(currAction == null && nextStateToLoad != null) {
            nextBlockOfActionsIs();
        }

    }

    public void setHeading(double heading) {
        targetHeading = heading;
    }

    public void driveForwardADistance(int distance) {
        distance = (int)(distance * 333.3);
        targetDistance_fb = distance;
    }

    public void driveRightADistance(int distance) {
        distance = (int)(distance * 333.3) * -1;
        targetDistance_lr = distance;
    }

    public void dropPixel(){
        dropPixel.setPosition(0.0);
        actionCompleteCommanderReadyForNextCommand();
    }

    public void theBIGSlap(){
        leftArm.setPosition(0.6);
        sleep(300);
        leftClaw.setPosition(0.0);
        sleep(750);
        leftArm.setPosition(0.3);
        actionCompleteCommanderReadyForNextCommand();
    }

    public void wait(int timeToWait){
        motor0_br.resetEncoder();
        motor1_bl.resetEncoder();
        motor2_fr.resetEncoder();
        motor3_fl.resetEncoder();
        resetMotors();
        currPower = 0.0;
        targetPower = 0.0;
        zeroDave_fb = Dave_fb.getCurrentPosition();
        zeroJeff_lr = Jeff_lr.getCurrentPosition();
        targetDistance_fb = 0;
        targetDistance_lr = 0;

        sleep(timeToWait);
        actionCompleteCommanderReadyForNextCommand();
    }

    public void driveToAprilTag(int idTarget) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        boolean weSawTheTagThisFrame = false;
        //We are on the RED side of the field
        // Codes go 4 - 5 - 6
        //
        // Can we see the target?
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == idTarget) {
                detectedTag = detection;
                framesSinceLastTagSighting = 0;
                weSawTheTagThisFrame = true;
            }
        }

        if (!weSawTheTagThisFrame) {
            framesSinceLastTagSighting += 1;
            if (framesSinceLastTagSighting > 5) {
                targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
                targetDistance_fb = (Dave_fb.getCurrentPosition() - zeroDave_fb);
                detectedTag = null;
            }
        }

        telemetry.addData("Navigating to tag", idTarget);
        // if so drive in that direction
        if (detectedTag != null) {
            if (centeredOnTag && !tooCloseToDetectSlideInTime) { //move in close
                //time to sneak up to it
                // forward distance should be 3
                telemetry.addData("centred on tag", idTarget);
                currentMotionType = CurrentMotionType.FORWARD;
                targetDistance_fb = (Dave_fb.getCurrentPosition() - zeroDave_fb);
                if (Math.abs(detectedTag.ftcPose.bearing + 25.0) > 10) {
                    centeredOnTag = false;
                    targetDistance_fb = (Dave_fb.getCurrentPosition() - zeroDave_fb);
                }
                if (detectedTag.ftcPose.y > 5) {
                    targetDistance_fb -= 300;
                    telemetry.addData("Jeff", (Jeff_lr.getCurrentPosition() - zeroJeff_lr));
                } else if (detectedTag.ftcPose.y < 5 && !tooCloseToDetectSlideInTime){
                    targetDistance_fb -= 900; //final drive in too close for camera to be reliable
                    tooCloseToDetectSlideInTime = true;
                }
            } else if (!centeredOnTag){ //get centred
                if (detectedTag.ftcPose.bearing > -23.0) {
                    // we are to the right of the tag, move left
                    targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
                    currentMotionType = CurrentMotionType.RIGHT;
                    targetDistance_lr -= 150;
                } else if (detectedTag.ftcPose.bearing < -27.0) {
                    targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
                    currentMotionType = CurrentMotionType.RIGHT;
                    targetDistance_lr += 150;
                } else {
                    centeredOnTag = true;
                    targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
                }
            } else {
                //ready to slap after movement is complete; movement code will handle that
            }

            // I am lost :(
            telemetry.addData("I don't see the tag :(", idTarget);
        }
    }


    public void figureOutMovement() {
        /* parameters! */
        double rotationCoefficient = 1.3;
        double linearCoefficient = 0.01;

        int rotCeil = 120;
        int rotFloor = 10;
        double rotAddAngle = 0.35;

        int driveCeil = 120;
        int driveFloor = 35;
        int driveAddDist = 40;

        int driftCeil = 120;
        int driftFloor = 20;
        int driftAddDist = 30;
        /* end */


        this.resetMotors();

        double currHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double toGoHeading = (targetHeading - currHeading + 180) % 360 - 180;
        toGoHeading = toGoHeading < -180 ? toGoHeading + 360 : toGoHeading;

        int toAdd = (int)(toGoHeading*rotationCoefficient);

        if (Math.abs(toAdd) > rotCeil) {
            toAdd = (int)Math.signum(toAdd) * rotCeil;
        } else if (Math.abs(toAdd) < rotFloor) {
            toAdd = (int)Math.signum(toAdd) * rotFloor;
        }

        if ( Math.abs(toGoHeading) > rotAddAngle ) {
            this.rotate(toAdd);
        }

        double distanceToTarget_fb = 0.0;
        double distanceToTarget_lr = 0.0;

        int fbCeil;
        int fbFloor;
        int fbAddDist;

        int lrCeil;
        int lrFloor ;
        int lrAddDist;

        if(currentMotionType != CurrentMotionType.ROTATE) {

            if (currentMotionType == CurrentMotionType.RIGHT) {
                fbCeil = driftCeil;
                fbFloor = driftFloor;
                fbAddDist = driftAddDist;

                lrCeil = driveCeil;
                lrFloor = driveFloor;
                lrAddDist = driveAddDist;
            } else if (currentMotionType == CurrentMotionType.FORWARD)  {
                fbCeil = driveCeil;
                fbFloor = driveFloor;
                fbAddDist = driveAddDist;

                lrCeil = driftCeil;
                lrFloor = driftFloor;
                lrAddDist = driftAddDist;
            } else {
                fbCeil = 0;
                fbFloor = 0;
                fbAddDist = 0;

                lrCeil = 0;
                lrFloor = 0;
                lrAddDist = 0;
            }

            distanceToTarget_fb = targetDistance_fb - (Dave_fb.getCurrentPosition() - zeroDave_fb);
            distanceToTarget_lr = targetDistance_lr - (Jeff_lr.getCurrentPosition() - zeroJeff_lr);

            toAdd = (int) (distanceToTarget_fb * linearCoefficient);
            if (Math.abs(toAdd) > fbCeil) {
                toAdd = (int) Math.signum(toAdd) * fbCeil;
            } else if (Math.abs(toAdd) < fbFloor) {
                toAdd = (int) Math.signum(toAdd) * fbFloor;
            }

            if (Math.abs(distanceToTarget_fb) > fbAddDist) {
                this.forwardBackDistance(toAdd);
            }


            toAdd = (int) (distanceToTarget_lr * linearCoefficient);
            if (Math.abs(toAdd) > lrCeil) {
                toAdd = (int) Math.signum(toAdd) * lrCeil;
            } else if (Math.abs(toAdd) < lrFloor) {
                toAdd = (int) Math.signum(toAdd) * lrFloor;
            }

            if (Math.abs(distanceToTarget_lr) > lrAddDist) {
                this.leftRightDistance(toAdd);
            }

        }

        if ( targetHeadingLastItr == toGoHeading
                && targetDistance_fbLastItr == distanceToTarget_fb
                && targetDistance_lrLastItr == distanceToTarget_lr

        ) {
            frameCountInStandstill += 1;
            if (frameCountInStandstill > 1) {
                frameCountInStandstill = 0;
                actionCompleteCommanderReadyForNextCommand();
            }
        } else {
            targetHeadingLastItr = toGoHeading;
            targetDistance_fbLastItr = distanceToTarget_fb;
            targetDistance_lrLastItr = distanceToTarget_lr;
            frameCountInStandstill = 0;
        }


        if (currentMotionType == CurrentMotionType.RIGHT) {
            this.setPower(toGoHeading, distanceToTarget_lr, distanceToTarget_fb);
        } else {
            this.setPower(toGoHeading, distanceToTarget_fb, distanceToTarget_lr);
        }

    }

    public void rotate(int amount) {
        motor0target += amount;
        motor1target += amount;
        motor2target += amount;
        motor3target += amount;
        motor0_br.setTargetPosition(motor0target); //set to target pos
        motor1_bl.setTargetPosition(motor1target);
        motor2_fr.setTargetPosition(motor2target);
        motor3_fl.setTargetPosition(motor3target);
    }

    public void forwardBackDistance(int amount) {
        motor0target += amount;
        motor1target -= amount;
        motor2target += amount;
        motor3target -= amount;
        motor0_br.setTargetPosition(motor0target); //set to target pos
        motor1_bl.setTargetPosition(motor1target);
        motor2_fr.setTargetPosition(motor2target);
        motor3_fl.setTargetPosition(motor3target);
    }

    public void leftRightDistance(int amount) {
        motor0target -= amount;
        motor1target -= amount;
        motor2target += amount;
        motor3target += amount;
        motor0_br.setTargetPosition(motor0target); //set to target pos
        motor1_bl.setTargetPosition(motor1target);
        motor2_fr.setTargetPosition(motor2target);
        motor3_fl.setTargetPosition(motor3target);
    }

    public void resetMotors() {
        motor0target = motor0_br.getCurrentPosition();
        motor1target = motor1_bl.getCurrentPosition();
        motor2target = motor2_fr.getCurrentPosition();
        motor3target = motor3_fl.getCurrentPosition();
        motor0_br.setTargetPosition(motor0target); //set to target pos
        motor1_bl.setTargetPosition(motor1target);
        motor2_fr.setTargetPosition(motor2target);
        motor3_fl.setTargetPosition(motor3target);
    }

    public void setPower(double distToRotation, double distToLinear, double distToDrift) {

        if (Math.abs(distToRotation) > 0.35 || Math.abs(distToLinear) > 40.0 || Math.abs(distToDrift) > 40.0) {
            if (Math.abs(distToRotation) > 0.35) {
                targetPower = Math.abs(distToRotation / 80.0);
            } else if (Math.abs(distToDrift) > 40.0) {
                targetPower = Math.abs(distToDrift / 1000.0);
            }
            else if (Math.abs(distToLinear) > 40.0) {
                targetPower = Math.abs(distToLinear / 8000.0);
            }
            targetPower = Math.min(targetPower, 1.0);
            targetPower = Math.max(targetPower, 0.16);
        }
        else {
            targetPower = 0.0;
            currPower = 0.0;
        }

        if (targetPower > currPower) {
            currPower = currPower + 0.05;
        } else if (targetPower < currPower) {
            currPower = currPower - 0.04;
        }
        motor0_br.set(currPower); //set target power
        motor1_bl.set(currPower); //set target power
        motor2_fr.set(currPower); //set target power
        motor3_fl.set(currPower); //set target power



    }

    @Override
    public void runOpMode() throws InterruptedException {
        motor0_br = this.initializeMotor("br");
        motor1_bl = this.initializeMotor("bl");
        motor2_fr = this.initializeMotor("fr");
        motor3_fl = this.initializeMotor("fl");
        dropPixel = hardwareMap.get(Servo.class, "Front Gate");
        dropPixel.setPosition(0.15);
        leftArm = hardwareMap.get(Servo.class, "Left Arm");
        leftArm.setPosition(0.3);
        leftClaw = hardwareMap.get(Servo.class, "Left Claw");
        leftClaw.setPosition(1);


        Dave_fb = hardwareMap.get(DcMotor.class, "Dave");//forward back
        zeroDave_fb = Dave_fb.getCurrentPosition();

        Jeff_lr = hardwareMap.get(DcMotor.class, "Jeff");//left right
        zeroJeff_lr = Jeff_lr.getCurrentPosition();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

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

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Back Camera"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        // Auto Examples
        // this.addActionToQueue("faceHeading", 90); // face LEFT
        // this.addActionToQueue("driveForwardADistance", 16000);
        // this.addActionToQueue("driveRightADistance", 16000);

        /* PLACE AUTO COMMANDS HERE! */

        /* THIS DOES BACK STAGE WITH CENTER RED MONKE */
        placePixelInCentre.addActionToQueue("driveForwardADistance", 24);
        placePixelInCentre.addActionToQueue("dropPixel", 0);
        placePixelInCentre.addActionToQueue("driveForwardADistance", 4);
        placePixelInCentre.addActionToQueue("driveForwardADistance", -5);
        placePixelInCentre.addActionToQueue("faceHeading", 90);
        placePixelInCentre.addActionToQueue("driveForwardADistance", -12);
        placePixelInCentre.addActionToQueue("driveRightADistance", 12);
        placePixelInCentre.addActionToQueue("navigateToAprilTag", 5);
        placePixelInCentre.addActionToQueue("theBIGSlap", 0);
        placePixelInCentre.addActionToQueue("driveForwardADistance", 2);
        placePixelInCentre.addActionToQueue("driveRightADistance", -33);

        goToRightForCheck.addActionToQueue("driveForwardADistance", 6);
        goToRightForCheck.addActionToQueue("driveRightADistance", 11);

        placePixelOnRight.addActionToQueue("driveForwardADistance", 10);
        placePixelOnRight.addActionToQueue("driveRightADistance", 3);
        placePixelOnRight.addActionToQueue("dropPixel", 0);
        placePixelOnRight.addActionToQueue("driveForwardADistance", 2);
        //placePixelOnRight.addActionToQueue("wait", 0.5);
        placePixelOnRight.addActionToQueue("driveForwardADistance", -6);
        placePixelOnRight.addActionToQueue("faceHeading", 90);
        placePixelOnRight.addActionToQueue("driveForwardADistance", -12);
        placePixelOnRight.addActionToQueue("driveRightADistance", 15);
        placePixelOnRight.addActionToQueue("navigateToAprilTag", 6);
        placePixelOnRight.addActionToQueue("theBIGSlap", 0);
        placePixelOnRight.addActionToQueue("driveForwardADistance", 2);
        placePixelOnRight.addActionToQueue("driveRightADistance", -26);

        placePixelOnLeft.addActionToQueue("driveForwardADistance", 28);
        placePixelOnLeft.addActionToQueue("faceHeading", 90);
        placePixelOnLeft.addActionToQueue("dropPixel", 0);
        //placePixelOnLeft.addActionToQueue("wait", 0.5);
        placePixelOnLeft.addActionToQueue("driveForwardADistance", 8);
        placePixelOnLeft.addActionToQueue("navigateToAprilTag", 4);
        placePixelOnLeft.addActionToQueue("theBIGSlap", 0);
        placePixelOnLeft.addActionToQueue("driveForwardADistance", 2);
        placePixelOnLeft.addActionToQueue("driveRightADistance", -39);


        testBlock.addActionToQueue("navigateToAprilTag", 5);
        testBlock.addActionToQueue("theBIGSlap", 0);
        testBlock.addActionToQueue("driveRightADistance", -33);
        //Pixl arm up
        //drop pixl

        /*THIS DOES BACK STAGE WITH RIGHT AND RED MONKE
        this.addActionToQueue("driveForwardADistance", 4000);
        this.addActionToQueue("driveForwardADistance", 8000);
        this.addActionToQueue("dropPixel", 0);
        this.addActionToQueue("driveForwardADistance", 667);
        this.addActionToQueue("faceHeading", 90);
        // Detect Right tag(id 6)
        this.addActionToQueue("DriveForwardADistance", 8000);
        //pixelArm Up
        //Drop Pixel
        this.addActionToQueue("driveRightADistance",-2000 );*/
        /*
        this.addActionToQueue("driveForwardADistance", 9666);
        this.addActionToQueue("faceHeading", 90);
        this.addActionToQueue("driveForwardADistance", 5333);
        this.addActionToQueue("dropPixel", 0);
        this.addActionToQueue("driveForwardADistance", 18333);
        */
        //PIXEL UP
        //DROP Pxl
        //this.addActionToQueue("driveRightADistance", 7333);
        /* END AUTO COMMANDS */

        rightCheckForMonkey =  new StateFlowAbstract("rightCheckForMonkey", "monkeyIsVisible", placePixelOnRight, null, placePixelOnLeft, null);
        initialState = new StateFlowAbstract("initialState", "monkeyIsVisible", placePixelInCentre, null, goToRightForCheck, rightCheckForMonkey);

        //initialState = new StateFlowAbstract("testState", null, testBlock, null, null, null);

        nextStateToLoad = initialState;


        while (opModeInInit()) {

            telemetry.addData("isMonkeyVisible?", cameraPipeline.doISeeMonkey());
            telemetry.update();
        }

        waitForStart();

        nextBlockOfActionsIs();
        actionCompleteCommanderReadyForNextCommand();

        while (opModeIsActive()) {

            this.executeCurrAction();
            this.figureOutMovement();

            telemetry.addData("power?", currPower);
            telemetry.addData("FB", targetDistance_fb - (Dave_fb.getCurrentPosition() - zeroDave_fb));
            telemetry.addData("LR", targetDistance_lr - (Jeff_lr.getCurrentPosition() - zeroJeff_lr));
            telemetry.update();

        }

    }

}
