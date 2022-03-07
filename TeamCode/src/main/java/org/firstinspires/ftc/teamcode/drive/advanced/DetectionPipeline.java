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
    private volatile int average;
    private volatile int average2;
    private volatile int average3;
    private volatile int averagefin=0;
    private volatile TYPE type ;

    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    private Point[] topPoints = new Point[20];
    private Point[] botPoints = new Point[20];
    private Mat[]   allMats = new Mat[20];
    private boolean[] validZones = new boolean[64];
    @Override
    public void init(Mat input)
    {
        inputToCb(input);
        cols = input.cols();
        rows = input.rows();
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        int c = 0;

        for(int i = 0; i < cols;i+= cols / 3)
        {
            for(int j = 0; j < rows; j += rows / 3)
            {
                Point p1 = new Point(i,j);

                if(i + cols / 3 >= cols || j + rows / 3 >= rows) continue;

                Point p2 = new Point(i + cols / 3 - 1, j + rows / 3  - 1);

                Imgproc.rectangle(input, p1, p2, BLUE, 2);

                allMats[c] = Cb.submat(new Rect(p1,p2));

                topPoints[c] = p1;
                botPoints[c] = p2;

                validZones[c] = Core.mean(allMats[c]).val[0] < THRESHOLD;
                c++;

            }

        }

        return input;
    }

    public TYPE getType() {
        return type;
    }
    public boolean[] getValidZones() { return validZones; }
    public boolean isZoneValid(int zone) { return validZones[zone];};
    public int getAveragefin(){return averagefin;}
    public int getAverage1() {
        return average;
    }
    public int getAverage2() {
        return average2;
    }
    public int getAverage3() {
        return average3;
    }

    public enum TYPE {
        ZONE1, ZONE2,ZONE3,NO
    }
}
