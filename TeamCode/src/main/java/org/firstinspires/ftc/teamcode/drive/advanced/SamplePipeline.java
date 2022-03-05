package org.firstinspires.ftc.teamcode.drive.advanced;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SamplePipeline extends OpenCvPipeline {
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
    Mat Cb2 = new Mat();
    Mat Cb3 = new Mat();

    private volatile int average;
    private volatile int average2;
    private volatile int average3;
    private volatile int averagefin=0;
    private volatile TYPE type ;

    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }
    private void inputToCb2(Mat input) {
        Imgproc.cvtColor(input, YCrCb2, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb2, Cb2, 2);
    }
    private void inputToCb3(Mat input) {
        Imgproc.cvtColor(input, YCrCb3, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb3, Cb3, 2);
    }

    @Override
    public void init(Mat input) {
        inputToCb(input);
        inputToCb2(input);
        inputToCb3(input);

        Point topLeft = new Point(input.cols()*(0.5f/12f), input.rows()*(2f/12f));
        Point bottomRight = new Point(input.cols()*(3.5f/12f), input.rows()*(8f/12f));
        Point topLeft1 = new Point(input.cols()*(4.5f/12f), input.rows()*(2f/12f));
        Point bottomRight1 = new Point(input.cols()*(7.5f/12f), input.rows()*(8f/12f));
        Point topLeft2 = new Point(input.cols()*(8.5f/12f), input.rows()*(2f/12f));
        Point bottomRight2 = new Point(input.cols()*(11.5f/12f), input.rows()*(8f/12f));

        region1_Cb = Cb.submat(new Rect(topLeft, bottomRight));
        region2_Cb= Cb2.submat(new Rect(topLeft1,bottomRight1));
        region3_Cb= Cb2.submat(new Rect(topLeft2,bottomRight2));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        inputToCb2(input);
        inputToCb3(input);

        Point topLeft = new Point(input.cols()*(0.5f/12f), input.rows()*(2f/12f));
        Point bottomRight = new Point(input.cols()*(3.5f/12f), input.rows()*(8f/12f));
        Point topLeft1 = new Point(input.cols()*(4.5f/12f), input.rows()*(2f/12f));
        Point bottomRight1 = new Point(input.cols()*(7.5f/12f), input.rows()*(8f/12f));
        Point topLeft2 = new Point(input.cols()*(8.5f/12f), input.rows()*(2f/12f));
        Point bottomRight2 = new Point(input.cols()*(11.5f/12f), input.rows()*(8f/12f));

        average = (int) Core.mean(region1_Cb).val[0];
        average2 = (int) Core.mean(region2_Cb).val[0];
        average3 = (int) Core.mean(region3_Cb).val[0];

        Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);
        Imgproc.rectangle(input, topLeft1, bottomRight1, BLUE, 2);
        Imgproc.rectangle(input, topLeft2, bottomRight2, BLUE, 2);


          /*  if (average > THRESHOLD) {
                type = TYPE.ZONE3;
                averagefin = average;
            } else if (average2 > THRESHOLD){
                type = TYPE.ZONE1;
                averagefin=average2;
        }
             else
             if(average3>THRESHOLD){
                type=TYPE.ZONE2;
                 averagefin=average3;}
             else type=TYPE.NO;*/
        if(average>120 && average2>120 && average3>120)
        {
            type=TYPE.NO;
            zona=0;
        }
        else if(average < average2 && average < average3)
        {
            type = TYPE.ZONE1;
            averagefin = average;
            zona=1;
        }
        else if(average2 < average && average2 < average3)
        {
            type = TYPE.ZONE2;
            averagefin = average2;
            zona=2;
        }
        else if(average3 < average2 && average3 < average)
        {
            type = TYPE.ZONE3;
            averagefin = average3;
            zona=3;
        }





        return input;
    }

    public TYPE getType() {
        return type;
    }

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