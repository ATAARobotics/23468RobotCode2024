package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.sql.Struct;
import java.util.LinkedList;

enum CurrentMotionType {
    ROTATE,
    FORWARD,
    RIGHT,
    NO
}

class StateFlowAbstract {
    public String condition;
    public BlockOfActions nextBlockIfTrue;
    public StateFlowAbstract nextStateIfTrue;
    public BlockOfActions nextBlockIfFalse;
    public StateFlowAbstract nextStateIfFalse;
    public String name;

    public StateFlowAbstract(String name, String condition, BlockOfActions nextBlockIfTrue, StateFlowAbstract nextStateIfTrue, BlockOfActions nextBlockIfFalse, StateFlowAbstract nextStateIfFalse) {
        this.condition = condition;
        this.nextBlockIfTrue = nextBlockIfTrue;
        this.nextStateIfTrue = nextStateIfTrue;
        this.nextBlockIfFalse = nextBlockIfFalse;
        this.nextStateIfFalse = nextStateIfFalse;
        this.name = name;
    }

}

class BlockOfActions {
    public LinkedList<AutoAction> autoActionQueue = new LinkedList<AutoAction>();
    public String name;

    public BlockOfActions(String name) {
        this.name = name;
    }

    public void addActionToQueue(String actionToTake, double amt) {
        AutoAction a = new AutoAction(actionToTake, amt);
        autoActionQueue.add(a);
    }
}

class AutoAction {
    public String actionType = "";
    public double amt = 0.0;

    public AutoAction(String actionType, double amt) {
        this.actionType = actionType;
        this.amt = amt;
    }
}


class CraigCameraPipeline extends OpenCvPipeline
{
    // Best practice is to put working vars in the pipeline object, do not create local variables as that causes memory leaks. There is no automatic garbage collection here
    Mat YCrCb = new Mat();
    Mat colourMat = new Mat();
    Mat regionToSample;
    int avgColourOfRegion;
    boolean areWeDetectingBlue;
    int threshold;
    Scalar detectColour;

    static final Point regionToSampleTopLeftCorner = new Point(100,175);
    static final int regionWidth = 114;
    static final int regionHeight = 40;
    static final Point regionToSampleTopBtmRightCorner = new Point(
            regionToSampleTopLeftCorner.x + regionWidth,
            regionToSampleTopLeftCorner.y + regionHeight);

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREY = new Scalar(40, 40, 40);


    static final int bluenessThreshold = 135;
    static final int rednessThreshold = 5 ;



    public CraigCameraPipeline(boolean areWeDetectingBlue)
    {
        this.areWeDetectingBlue = areWeDetectingBlue;
        if (areWeDetectingBlue) {
            threshold = bluenessThreshold;
            detectColour = BLUE;
        } else {;
            threshold = rednessThreshold;
            detectColour = RED;
        }

    }


    public void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        if (areWeDetectingBlue) {
            Core.extractChannel(YCrCb, colourMat, 2);
        } else {
            Core.extractChannel(YCrCb, colourMat, 1);
        }
    }


    @Override
    public void init(Mat firstFrame)
    {
        //inputToCb(firstFrame);

        if (areWeDetectingBlue) {
            Core.extractChannel(firstFrame, colourMat, 3);
        } else {
            Core.extractChannel(firstFrame, colourMat, 0);//0 is red
        }
        regionToSample = colourMat.submat(new Rect(regionToSampleTopLeftCorner,  regionToSampleTopBtmRightCorner));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        //inputToCb(input);
        if (areWeDetectingBlue) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, colourMat, 2);//Best I found to figure out? worked great in bright conditions

            //Core.extractChannel(input, colourMat, 2);//worked better in dark light
        } else {
            Core.extractChannel(input, colourMat, 0);//0 is red 1 is green
        }
        Core.divide(regionToSample,new Scalar (15.0), regionToSample);
        Core.pow(regionToSample, 2, regionToSample);
        avgColourOfRegion = (int) Core.mean(regionToSample).val[0];

        // Extra visual aids, not required
        Imgproc.putText (
                colourMat,                          // Matrix obj of the image
                String.valueOf(avgColourOfRegion),          // Text to be added
                new Point(20, 20),               // point
                0,      // front face
                1,                               // front scale
                new Scalar(0, 0, 0)           // Scalar object for color
        );
        if (avgColourOfRegion >= threshold) {
            Imgproc.rectangle(
                    colourMat, // Buffer to draw on
                    regionToSampleTopLeftCorner, // First point which defines the rectangle
                    regionToSampleTopBtmRightCorner, // Second point which defines the rectangle
                    detectColour, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
        } else {
            Imgproc.rectangle(
                    colourMat, // Buffer to draw on
                    regionToSampleTopLeftCorner, // First point which defines the rectangle
                    regionToSampleTopBtmRightCorner, // Second point which defines the rectangle
                    GREY, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
        }

        return colourMat;

    }

    public boolean doISeeMonkey()
    {
        return avgColourOfRegion >= threshold;
    }
}
