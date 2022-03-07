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

    private static final int THRESHOLD = 112;
    private static int zona = 0;


    Mat region1_Cb;
    Mat region2_Cb;
    Mat region3_Cb;
    Mat YCrCb2=new Mat();
    Mat YCrCb3=new Mat();
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();


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
    public void init(Mat input) {
        inputToCb(input);
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
                Point p2 = new Point(i + cols / 3 - 1, j + rows / 3  - 1);

                Imgproc.rectangle(input, p1, p2, BLUE, 2);
                allMats[c] = Cb.submat(new Rect(p1,p2));
                topPoints[c] = p1;
                botPoints[c] = p2;

                if(Core.mean(allMats[c]).val[0] < 120)
                    validZones[c] = true;
                else
                    validZones[c] = false;
            }
            c++;
        }
        /*Point topLeft = new Point(input.cols()*(0.5f/12f), input.rows()*(2f/12f));
        Point bottomRight = new Point(input.cols()*(3.5f/12f), input.rows()*(8f/12f));
        Point topLeft1 = new Point(input.cols()*(4.5f/12f), input.rows()*(2f/12f));
        Point bottomRight1 = new Point(input.cols()*(7.5f/12f), input.rows()*(8f/12f));
        Point topLeft2 = new Point(input.cols()*(8.5f/12f), input.rows()*(2f/12f));
        Point bottomRight2 = new Point(input.cols()*(11.5f/12f), input.rows()*(8f/12f));

        Point topLeft2 = new Point(input.cols()*(0.5f/12f), input.rows()*(2f/12f));
        Point bottomRight = new Point(input.cols()*(3.5f/12f), input.rows()*(8f/12f));
        Point topLeft12 = new Point(input.cols()*(4.5f/12f), input.rows()*(2f/12f));
        Point bottomRight2 = new Point(input.cols()*(7.5f/12f), input.rows()*(8f/12f));
        Point topLeft22 = new Point(input.cols()*(8.5f/12f), input.rows()*(2f/12f));
        Point bottomRight22 = new Point(input.cols()*(11.5f/12f), input.rows()*(8f/12f));

        Point topLeft3 = new Point(input.cols()*(0.5f/12f), input.rows()*(2f/12f));
        Point bottomRight3 = new Point(input.cols()*(3.5f/12f), input.rows()*(8f/12f));
        Point topLeft13 = new Point(input.cols()*(4.5f/12f), input.rows()*(2f/12f));
        Point bottomRight13 = new Point(input.cols()*(7.5f/12f), input.rows()*(8f/12f));
        Point topLeft23 = new Point(input.cols()*(8.5f/12f), input.rows()*(2f/12f));
        Point bottomRight23 = new Point(input.cols()*(11.5f/12f), input.rows()*(8f/12f));*/

        /*for(int i = 0; i < input.rows();i+=160)
        {
            for(int j = 0; j < input.cols();j+=226)
            {
                Point p1 = new Point(j,i);
                Point p2 = new Point(j + 225,i + 159);

                Point p3 = new Point(i,j);
                Point p4 = new Point(i  + 159,j + 225);

                Imgproc.rectangle(input, p1, p2, BLUE, 2);
                allMats[c] = Cb.submat(new Rect(p3,p4));
                topPoints[c] = p1;
                botPoints[c] = p2;

                if(Core.mean(allMats[c]).val[0] < 120)
                    validZones[c] = true;
                else
                    validZones[c] = false;
            }
            c++;
        }*/
//        average = (int) Core.mean(region1_Cb).val[0];
//        average2 = (int) Core.mean(region2_Cb).val[0];
//        average3 = (int) Core.mean(region3_Cb).val[0];
//
//        Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);
//        Imgproc.rectangle(input, topLeft1, bottomRight1, BLUE, 2);
//        Imgproc.rectangle(input, topLeft2, bottomRight2, BLUE, 2);
//
//
//        if(average>120 && average2>120 && average3>120)
//        {
//            type=TYPE.NO;
//            zona=0;
//        }
//        else if(average < average2 && average < average3)
//        {
//            type = TYPE.ZONE1;
//            averagefin = average;
//            zona=1;
//        }
//        else if(average2 < average && average2 < average3)
//        {
//            type = TYPE.ZONE2;
//            averagefin = average2;
//            zona=2;
//        }
//        else if(average3 < average2 && average3 < average)
//        {
//            type = TYPE.ZONE3;
//            averagefin = average3;
//            zona=3;
//        }
//
//
//


        return input;
    }

    public TYPE getType() {
        return type;
    }
    public boolean[] getValidZones() { return validZones; }
    public boolean isZoneValid(int zone) { return validZones[zone];};
    public int getAverage() {
        return zona;
    }
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
