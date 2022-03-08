package org.firstinspires.ftc.teamcode.drive.advanced;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeline extends OpenCvPipeline {
    private static final Scalar BLUE = new Scalar(0, 0, 255);

    private static final int THRESHOLD = 120;

    Mat YCrCb = new Mat();
    Mat Cb = new Mat();\
    private static int cols,rows;
    private static int numberOfElements = 0;

    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    private Point[] topPoints = new Point[20];
    private Point[] botPoints = new Point[20];
    private Point[] centerPoints = new Point[20];
    private Mat[]   allMats = new Mat[20];
    private boolean[] validZones = new boolean[64];
    
    @Override
    public void init(Mat input)
    {
        inputToCb(input);
        
        cols = input.cols();
        rows = input.rows();
        
        for(int i = 0; i < cols;i+= cols / 3)
        {
            for(int j = 0; j < rows; j += rows / 3)
            {
                int topLeftX = i;
                int topLeftY = j;
                
                int botRightX = i + cols / 3 - 1;
                int botRightY = j + rows / 3 - 1;
                
                if(botRightX >= cols || botRightY >= rows) continue;
                
                Point p1 = new Point(i,j);
                             
                Point p2 = new Point(botRightX, botRightY);
                
                Point center = new Point((topLeftX + botRightX) / 2 , (topLeftY + botRightY) / 2);

                topPoints[numberOfElements] = p1;
                
                botPoints[numberOfElements] = p2;
                
                centerPoints[numberOfElements] = center;
                
                numberOfElements++;
            }

        }
    }
    
    @Override
    public Mat processFrame(Mat input) 
    {
        inputToCb(input);

        for(int i = 0; i < numberOfElements;i++)
        {
            allMats[i] = Cb.submat(new Rect(topPoints[i],botPoints[i]));

            String zoneName = "Zone " + i + 1;
            
            Imgproc.putText(input, zoneName, centerPoints[i] , Imgproc.FONT_ITALIC , 1, BLUE,1);
                
            Imgproc.rectangle(input, topPoints[i], botPoints[i], BLUE, 2);

            validZones[i] = Core.mean(allMats[i]).val[0] < THRESHOLD;
        }

        return input;
    }
  
    public boolean[] getValidZones() { return validZones; }
    public boolean isZoneValid(int zone) { return validZones[zone];};
}
