package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "Reconnstruct_Auto", group = "Concept")
public class Reconnstruct_Auto extends LinearOpMode {
    public Motor motor0_br = null; //Back right
    public Motor motor1_bl = null; //Back left
    //public Motor motor2_fr = null; //front right
    //public Motor motor3_fl = null; //front left

    public double targetPower = 0.0;
    public double currPower = 0.0;

    public double targetHeading = 0.0;
    public int targetDistance_fb = 0;

    public double targetHeadingLastItr = -1.0;
    public double targetDistance_fbLastItr = -1;

    //public int frameCountInStandstill = 0;

    public CurrentMotionType currentMotionType = CurrentMotionType.ROTATE;

    public int motor0target = 0;
    public int motor1target = 0;
    //public int motor2target = 0;
    //public int motor3target = 0;

    double endOfWaitTime;
    boolean isWaiting = false;
	//boolean eligibleForTransition = false;
    boolean doingFineAdjustments = false;

    //public DcMotor Dave_fb = null; //dave is forward/backwards, jeff is side to side
    //public int zeroDave_fb = 0;
    public int zero_motor0_br = 0;

    //public DcMotor Jeff_lr = null; //dave is forward/backwards, jeff is side to side
    //public int zeroJeff_lr;

    public IMU imu;
    public RevHubOrientationOnRobot orientationOnRobot;

    public BlockOfActions Do_Auto = new BlockOfActions("Do_Auto");


    public BlockOfActions testBlock = new BlockOfActions("testBlock");

    public StateFlowAbstract initialState = null;
    //public StateFlowAbstract leftCheckForMonkey = null;

    public AutoAction currAction = null;
    public BlockOfActions currBlock = null;
    public StateFlowAbstract nextStateToLoad = null;

    //public CraigCameraPipeline cameraPipeline = new CraigCameraPipeline(false); // -------------------------------------------------------------------CHANGE THIS TO DETECT RED/BLUE

    final double fx = 466.664;
    final double fy = 466.664;
    final double cx = 114.847;
    final double cy = 170.026;
    //public AprilTagProcessor aprilTag;
    //public VisionPortal visionPortal;
    //public AprilTagDetection detectedTag;
    //public int framesSinceLastTagSighting = 0;
    //public boolean centeredOnTag = false;
    //public boolean tooCloseToDetectSlideInTime = false;

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
            case "turn":
                currentMotionType = CurrentMotionType.ROTATE;
                this.setHeading(currAction.amt);
                break;
            case "forward":
                currentMotionType = CurrentMotionType.FORWARD;
                this.forward((int) currAction.amt);
                break;
                /*
            case "driveRightADistance":
                currentMotionType = CurrentMotionType.RIGHT;
                this.driveRightADistance((int) currAction.amt);
                break;
                 */
            case "dropPixel":
                currentMotionType = CurrentMotionType.NO;
                this.dropPixel();
                break;
            case "wait":
                wait((int) currAction.amt);
                break;
                /*
            case "navigateToAprilTag":
				//visionPortal.resumeStreaming();
                //sleep(500);
                driveToAprilTag((int) currAction.amt);
                break;
            case "theBIGSlap":
				visionPortal.stopStreaming();
                theBIGSlap();
                break;

                 */
        }
    }

    public boolean checkConditions() {
        if (nextStateToLoad.condition == null) {
            return true;
        }
        switch (nextStateToLoad.condition) {
            /*
            case "monkeyIsVisible":
                telemetry.addData("monkeyIsVisible", cameraPipeline.doISeeMonkey());
                return cameraPipeline.doISeeMonkey();
            case "secondConditionToSuppressWarning":
                return true;
             */
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
		currAction = currBlock.autoActionQueue.poll();											  
    }

    public void actionCompleteCommanderReadyForNextCommand() {
        motor0_br.resetEncoder();
        motor1_bl.resetEncoder();
        /*
        motor2_fr.resetEncoder();
        motor3_fl.resetEncoder();

         */
        resetMotors();
        currPower = 0.0;
        targetPower = 0.0;
        /*
        zeroDave_fb = Dave_fb.getCurrentPosition();
        zeroJeff_lr = Jeff_lr.getCurrentPosition();

         */
        targetDistance_fb = 0;
        //maintain target heading

        if (currBlock != null)  {
            currAction = currBlock.autoActionQueue.poll();
        }

        if(currAction == null && nextStateToLoad != null) {
            nextBlockOfActionsIs();
        }

		//eligibleForTransition = false;
        //frameCountInStandstill = 0;
        targetHeadingLastItr = -1;
        targetDistance_fbLastItr = -1;
    }

    public void setHeading(double heading) {
        targetHeading = heading;
    }

    public void forward(int distance) {
        distance = (int)(distance * 50);
        targetDistance_fb = distance;
    }

    /*
    public void driveRightADistance(int distance) {
        distance = (int)(distance * 333.3) * -1;
        targetDistance_lr = distance;
    }

     */

    // EXAMPLE
    public void dropPixel(){
        //dropPixel.setPosition(0.0);
        actionCompleteCommanderReadyForNextCommand();
    }


    public void wait(int timeToWait){
        motor0_br.resetEncoder();
        motor1_bl.resetEncoder();
        /*
        motor2_fr.resetEncoder();
        motor3_fl.resetEncoder();

         */
        resetMotors();
        currPower = 0.0;
        targetPower = 0.0;
        /*
        zeroDave_fb = Dave_fb.getCurrentPosition();
        zeroJeff_lr = Jeff_lr.getCurrentPosition();

         */
        targetDistance_fb = 0;

        sleep(timeToWait);
        actionCompleteCommanderReadyForNextCommand();
    }

    /*
    public void driveToAprilTag(int idTarget) {
		eligibleForTransition = false;

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
            if (framesSinceLastTagSighting > 2) {
                targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
                targetDistance_fb = (Dave_fb.getCurrentPosition() - zeroDave_fb);
                detectedTag = null;
            }
        }

        telemetry.addData("Navigating to tag", idTarget);
        // if so drive in that direction
        if (detectedTag != null) {
			telemetry.addLine(String.format("\n==== (ID %d) %s", detectedTag.id, detectedTag.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedTag.ftcPose.x, detectedTag.ftcPose.y, detectedTag.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedTag.ftcPose.pitch, detectedTag.ftcPose.roll, detectedTag.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedTag.ftcPose.range, detectedTag.ftcPose.bearing, detectedTag.ftcPose.elevation));
            																								 
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
                    targetDistance_fb -= 600; //final drive in too close for camera to be reliable //was 900
                    tooCloseToDetectSlideInTime = true;
					eligibleForTransition = true;							 
                }
            } else if (!centeredOnTag){ //get centred
                if (detectedTag.ftcPose.bearing > -23.0) {
                    // we are to the right of the tag, move left
                    targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
                    currentMotionType = CurrentMotionType.RIGHT;
                    targetDistance_lr -= 300;
                } else if (detectedTag.ftcPose.bearing < -27.0) {
                    targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
                    currentMotionType = CurrentMotionType.RIGHT;
                    targetDistance_lr += 300;
                } else {
                    centeredOnTag = true;
                    targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
                }
           } else {
                eligibleForTransition = true;
                //ready to slap after movement is complete; movement code will handle that
            }
        } else {
            // I am lost :(
            telemetry.addData("I don't see the tag :(", idTarget);
            targetDistance_lr = (Jeff_lr.getCurrentPosition() - zeroJeff_lr);
            targetDistance_fb = (Dave_fb.getCurrentPosition() - zeroDave_fb);
            if (framesSinceLastTagSighting > 5) {
                //its lost hopefully we are close, drop a pixel and go hide
                eligibleForTransition = true;
            }
        }

    }

     */


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

            if (currentMotionType == CurrentMotionType.FORWARD)  {
                fbCeil = driveCeil;
                fbFloor = driveFloor;
                fbAddDist = driveAddDist;

            } else {
                fbCeil = 0;
                fbFloor = 0;
                fbAddDist = 0;

            }


            distanceToTarget_fb = targetDistance_fb - (motor0_br.getCurrentPosition() - zero_motor0_br);



            toAdd = (int) (distanceToTarget_fb * linearCoefficient);
            if (Math.abs(toAdd) > fbCeil) {
                toAdd = (int) Math.signum(toAdd) * fbCeil;
            } else if (Math.abs(toAdd) < fbFloor) {
                toAdd = (int) Math.signum(toAdd) * fbFloor;
            }

            if (Math.abs(distanceToTarget_fb) > fbAddDist) {
                this.forwardBackDistance(toAdd);
            }

        }

        telemetry.addData("HD", toGoHeading);
        telemetry.addData("FB", distanceToTarget_fb);
        double TOLERANCE = 80;
        double ROTTOLERANCE = 0.5;
        if ( currAction != null
                && Math.abs(toGoHeading) < ROTTOLERANCE
                && Math.abs(distanceToTarget_fb) < TOLERANCE // close to dest

        ) {
            //frameCountInStandstill = 0;
            //eligibleForTransition = false;
            targetHeadingLastItr = -1.0;
            targetDistance_fbLastItr = -1;
            actionCompleteCommanderReadyForNextCommand();

        } else if ( targetHeadingLastItr == toGoHeading // Stall Protection
                && targetDistance_fbLastItr == distanceToTarget_fb
        ) {
            targetHeadingLastItr = -1.0;
            targetDistance_fbLastItr = -1;
            actionCompleteCommanderReadyForNextCommand();

        } else {

            targetHeadingLastItr = toGoHeading;
            targetDistance_fbLastItr = distanceToTarget_fb;
									   
        }

        //this.setPower(toGoHeading, distanceToTarget_fb);
        motor0_br.set(0.5); //set target power
        motor1_bl.set(0.5); //set target power


    }

    public void rotate(int amount) {
        motor0target += amount;
        motor1target += amount;
        /*
        motor2target += amount;
        motor3target += amount;

         */
        motor0_br.setTargetPosition(motor0target); //set to target pos
        motor1_bl.setTargetPosition(motor1target);
        /*
        motor2_fr.setTargetPosition(motor2target);
        motor3_fl.setTargetPosition(motor3target);

         */
    }

    public void forwardBackDistance(int amount) {
        motor0target += amount;
        motor1target -= amount;
        /*
        motor2target += amount;
        motor3target -= amount;

         */
        motor0_br.setTargetPosition(motor0target); //set to target pos
        motor1_bl.setTargetPosition(motor1target);
        /*
        motor2_fr.setTargetPosition(motor2target);
        motor3_fl.setTargetPosition(motor3target);

         */
    }

    /*
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
    */

    public void resetMotors() {
        motor0target = motor0_br.getCurrentPosition();
        motor1target = motor1_bl.getCurrentPosition();
        /*
        motor2target = motor2_fr.getCurrentPosition();
        motor3target = motor3_fl.getCurrentPosition();

         */
        motor0_br.setTargetPosition(motor0target); //set to target pos
        motor1_bl.setTargetPosition(motor1target);
        /*
        motor2_fr.setTargetPosition(motor2target);
        motor3_fl.setTargetPosition(motor3target);

         */
    }

    /*
    public void setPower(double distToRotation, double distToLinear) {

		if(currentMotionType != CurrentMotionType.ROTATE) {
            if (Math.abs(distToRotation) > 0.35 || Math.abs(distToLinear) > 40.0) {

                targetPower = Math.min(targetPower, 1.0);
                targetPower = Math.max(targetPower, 0.16);

            } else {
                targetPower = 0.0;
                currPower = 0.0;
            }

            double step = 0.05;
            if (currAction != null && currAction.turbo) {
                if (distToLinear > 2500) {
                    targetPower = 0.75;
                }
            }
													 
								 
            if (targetPower > currPower) {
                currPower = currPower + step;
            } else if (targetPower < currPower) {
                currPower = currPower - step;
            }

        } else {
            if (Math.abs(distToRotation) > 0.35) {
                targetPower = Math.abs(distToRotation / 90.0);
                targetPower = Math.min(targetPower, 1.0);
                targetPower = Math.max(targetPower, 0.3);
            } else {
                targetPower = 0.0;
            }
            currPower = targetPower;
        }

        telemetry.addData("power", targetPower);
        motor0_br.set(currPower); //set target power
        motor1_bl.set(currPower); //set target power


    }
    */



    @Override
    public void runOpMode() throws InterruptedException {
        motor0_br = this.initializeMotor("br");
        motor1_bl = this.initializeMotor("bl");

        zero_motor0_br = motor0_br.getCurrentPosition();


        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /*
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
        */

        // Auto Examples
        // this.addActionToQueue("turn", 90); // face LEFT
        // this.addActionToQueue("forward", 16000);

        Do_Auto.addActionToQueue("forward", -12);
        Do_Auto.addActionToQueue("turn", -90);

        initialState = new StateFlowAbstract("pixelinplate", null, Do_Auto, null, null, null);
        //initialState = new StateFlowAbstract("testState", null, testBlock,// null, null, null);

        nextStateToLoad = initialState;


        while (opModeInInit()) {
            /*
            telemetry.addData("isMonkeyVisible?", cameraPipeline.doISeeMonkey());
			telemetry.addData("Front Camera State", frontCamera.getFps() );
            telemetry.addData("Rear Camera State", visionPortal.getCameraState() );

            */
            telemetry.addData("IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("motor pos: ", zero_motor0_br);
            telemetry.update();
        }

        waitForStart();

        nextBlockOfActionsIs();
        //actionCompleteCommanderReadyForNextCommand();

        while (opModeIsActive()) {

            this.executeCurrAction();
            this.figureOutMovement();
			if ( currAction != null) {
                telemetry.addData("state", currAction.actionType );
            } else {
                telemetry.addLine("noActions" );
            }

            //telemetry.addData("isMonkeyVisible?", cameraPipeline.doISeeMonkey());

																									  
            telemetry.update();


        }

    }

}
