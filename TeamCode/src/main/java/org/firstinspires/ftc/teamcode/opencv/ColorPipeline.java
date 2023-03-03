package org.firstinspires.ftc.teamcode.opencv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorPipeline extends OpenCvPipeline {
    ArrayList<MatOfPoint> orangeContours = new ArrayList<>();
    ArrayList<MatOfPoint> greenContours = new ArrayList<>();
    ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
    ArrayList<MatOfPoint> yellowContours = new ArrayList<>();
    SignalColor color = SignalColor.INACTIVE;
    double maxArea;
    Double yellowLocation = null;
    Double yellowSize;
    Telemetry telemetry;

    public ColorPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        //input = input.submat(300,450,100,300);

        Scalar orangeLower = new Scalar(3,25,25);
        Scalar orangeUpper = new Scalar(12,255,255);

        Scalar greenLower = new Scalar(62,10,10);
        Scalar greenUpper = new Scalar(83,175,175);

        Scalar purpleLower = new Scalar(150,10,10);
        Scalar purpleUpper = new Scalar(170,255,255);

        Scalar yellowLower = new Scalar(15,75,150);
        Scalar yellowUpper = new Scalar(25,230,255);

        double orangeArea = findColorContourArea(input, orangeContours, orangeLower, orangeUpper, new Scalar(255, 100, 0));
        double greenArea = findColorContourArea(input, greenContours, greenLower, greenUpper, new Scalar(0, 255, 0));
        double purpleArea = findColorContourArea(input, purpleContours, purpleLower, purpleUpper, new Scalar(255, 0, 255));
        double yellowPosition = getColorPosition(input,yellowContours, yellowLower, yellowUpper, new Scalar(255, 255, 0));

        //pick the color with the largest area
        maxArea = Math.max(Math.max(purpleArea,orangeArea),greenArea);

        if(orangeArea > purpleArea && orangeArea > greenArea) {
            color = SignalColor.ORANGE;
        } else if(greenArea > purpleArea) {
            color = SignalColor.GREEN;
        } else if (purpleArea > 0) {
            color = SignalColor.PURPLE;
        } else {
            color = SignalColor.UNSET;
        }
        if(yellowPosition > 0) {
            yellowLocation = yellowPosition;
        }else{
            yellowLocation = null;
        }


//        telemetry.addData("Color", color);
//        telemetry.addData("Orange Area", orangeArea);
//        telemetry.addData("Green Area", greenArea);
//        telemetry.addData("Purple Area", purpleArea);
//        telemetry.addData("Yellow Location", yellowPosition);
//        telemetry.update();
        return input;
    }

    private double findColorContourArea(Mat input, ArrayList<MatOfPoint> contours, Scalar lower, Scalar upper, Scalar color){
        Mat workingMat = new Mat();
        //convert to HSV
        Imgproc.cvtColor(input,workingMat,Imgproc.COLOR_RGB2HSV);
        //filter the mat
        Core.inRange(workingMat,lower,upper,workingMat);
        //morph the mat (remove noise)
        Imgproc.morphologyEx(workingMat,workingMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        //find the contours
        contours.clear();
        Imgproc.findContours(workingMat, contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        MatOfPoint workingContour = null;
        //find the largest contour
        int index = 0;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(workingContour == null){
                    workingContour = newContour;
                    index =i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(workingContour)){
                    workingContour = newContour;
                    index = i;
                }
            }
        }
        //release the mat
        workingMat.release();
        //add the contour to the image
        try {
            Imgproc.drawContours(input,contours,index,color);
            return Imgproc.contourArea(workingContour);
        }catch(Exception ignored){
            //System.out.println("Orange Contour not found");
            return 0;
        }

    }

    private double getColorPosition(Mat input, ArrayList<MatOfPoint> contours, Scalar lower, Scalar upper, Scalar color){
        Mat workingMat = new Mat();
        //convert to HSV
        Imgproc.cvtColor(input,workingMat,Imgproc.COLOR_RGB2HSV);
        //filter the mat
        Core.inRange(workingMat,lower,upper,workingMat);
        //morph the mat (remove noise)
        Imgproc.morphologyEx(workingMat,workingMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        //find the contours
        contours.clear();
        Imgproc.findContours(workingMat, contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        MatOfPoint workingContour = null;
        //find the largest contour
        int index = 0;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(workingContour == null){
                    workingContour = newContour;
                    index =i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(workingContour)){
                    workingContour = newContour;
                    index = i;
                }
            }
        }
        //release the mat
        workingMat.release();
        //add the contour to the image
        try {
            Imgproc.drawContours(input,contours,index,color);
            yellowSize = Imgproc.contourArea(workingContour);
            return Imgproc.boundingRect(workingContour).x + Imgproc.boundingRect(workingContour).width/2.0;
        }catch(Exception ignored){
            //System.out.println("Orange Contour not found");
            return -1;
        }

    }
    public double getMaxArea(){
        return maxArea;
    }

    public SignalColor getColor() {
        return color;
    }

    public double getYellowArea(){
        return yellowSize==null?-1:yellowSize;
    }
    public double getYellowLocation() {
        return yellowLocation ==null? 0:yellowLocation;
    }
}