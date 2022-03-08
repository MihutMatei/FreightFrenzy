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
    Mat Cb = new Mat();
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

        for(int i = 0; i < 64;i++)
            validZones[i] = false;
    }
    
    @Override
    public Mat processFrame(Mat input) 
    {
        inputToCb(input);
        numberOfElements = 0;
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

                Point center = new Point((topLeftX + botRightX) / 2 - 25, (topLeftY + botRightY) / 2);

                allMats[numberOfElements] = Cb.submat(new Rect(p1,p2));

                String zoneName = "Zone " + (numberOfElements + 1);

                Imgproc.putText(input, zoneName, center , Imgproc.FONT_ITALIC , 1, BLUE,1);

                Imgproc.rectangle(input, p1, p2, BLUE, 2);

                validZones[numberOfElements] = Core.mean(allMats[numberOfElements]).val[0] < THRESHOLD;

                numberOfElements++;
            }

        }

        return input;
    }
  
    public boolean[] getValidZones() { return validZones; }
    public boolean isZoneValid(int zone) { return validZones[zone];};
}
