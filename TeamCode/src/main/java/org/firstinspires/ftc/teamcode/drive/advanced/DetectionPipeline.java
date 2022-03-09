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

    Mat luminosityMat = new Mat();
    Mat extractionMat = new Mat();

    private static final int gridSize = 3;
    private static int cols,rows;
    private static int numberOfElements = 0;

    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, extractionMat, Imgproc.COLOR_RGB2YCrCb); // convert rgb to chroma and luminosity

        Core.extractChannel(extractionMat, luminosityMat, 2);
    }

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
        for(int i = 0; i < cols;i+= cols / gridSize)
        {
            for(int j = 0; j < rows; j += rows / gridSize)
            {
                int topLeftX = i;
                int topLeftY = j;

                int botRightX = i + cols / gridSize - 1;
                int botRightY = j + rows / gridSize - 1;

                if(botRightX >= cols || botRightY >= rows) continue;

                Point p1 = new Point(i,j);

                Point p2 = new Point(botRightX, botRightY);

                Point center = new Point((topLeftX + botRightX) / 2 - 25, (topLeftY + botRightY) / 2);

                allMats[numberOfElements] = luminosityMat.submat(new Rect(p1,p2));

                String zoneName = "Zone " + (numberOfElements + 1);

                Imgproc.putText(input, zoneName, center , Imgproc.FONT_ITALIC , 1, BLUE,1);

                Imgproc.rectangle(input, p1, p2, BLUE, 2);

                validZones[numberOfElements] = Core.mean(allMats[numberOfElements]).val[0] < THRESHOLD;

                numberOfElements++;
            }

        }

        return input;
    }

   // public boolean[] getValidZones() { return validZones; }

    public ZoneType getZoneType(int zone) // zone parameter IS index 1 based
    {
        if(zone <= gridSize)
            return ZoneType.E_LEFT;
        else if(zone <= gridSize * 2)
            return ZoneType.E_CENTER;
        else
            return ZoneType.E_RIGHT;
    }

    public int getDuckZone()
    {
        int bestZone = 0;
        double bestAverage = 10000;
        for(int i = 0; i < allMats.length;i++) {
            if(!validZones[i]) continue;

            ZoneType zoneType = getZoneType(i + 1);

            double current_avg = Core.mean(allMats[i]).val[0];

            if(current_avg < bestAverage)
            {
                bestAverage = current_avg;
                bestZone = i;
            }
        }

        return bestZone;
    }
    public int getBestZone(ZoneType preferredZone) { // index 1 based zones
        int bestZone = 0;
        boolean foundCenterZone = false;
        boolean foundPrefferedZone = false;

        for(int i = 0; i < allMats.length;i++) {
            if(!validZones[i]) continue;

            ZoneType zoneType = getZoneType(i + 1);

            if(zoneType == ZoneType.E_CENTER)
            {
                foundCenterZone = true;
                bestZone = Math.max(bestZone , i + 1);
            }
            else if(!foundCenterZone && zoneType == preferredZone)
            {
                bestZone = Math.max(bestZone, i + 1);
                foundPrefferedZone = true;
            }
            else if(!foundCenterZone && !foundPrefferedZone)
            {
                bestZone = Math.max(bestZone, i + 1);
            }
        }

        return bestZone;
    }

    public boolean isZoneValid(int zone) { return validZones[zone]; };

    public enum ZoneType
    {
        E_NONE,E_LEFT,E_RIGHT,E_CENTER
    }

}
