package org.firstinspires.ftc.teamcode.drive.advanced;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeline extends OpenCvPipeline {
    private int x = 0,y = 0; // x and y relative to center
    private int average_x = 0,average_y = 0;
    private int rows,cols;
    private int nr_stanga = 0, nr_dreapta = 0;
    private E_SIDE side = E_SIDE.NONE;

    @Override
    public void init(Mat input) {
        rows = input.rows();
        cols = input.cols();
    }

    @Override
    public Mat processFrame(Mat input) {
        for(int i = 0; i < rows;i++)
        {
            for(int j = 0; j < cols;j++)
            {
                double[] color_arr = input.get(i, j);

                double red = color_arr[0];
                double green = color_arr[1];
                double blue = color_arr[2];

                boolean valid_color = false; // aici trebuie sa calculezi cat de aproape e culoarea actuala de cea pe care o cautam
                // si daca e ok setam pe true testati voi idk

                if(!valid_color) continue; // trecem la urmatorul pixel daca nu e buna culoarea

                if(i < rows / 2)
                    nr_stanga++;
                else
                    nr_dreapta++;

                // calculam average x si y pt culoarea gasita ca sa avem centrul obiectului
                average_x += i;
                average_y += j;
            }
        }

        if(nr_stanga > nr_dreapta)
        {
            side = E_SIDE.LEFT;

            // asta e pozitia obiectului dar pe noi ne intereseaza poz fata de centru pozei
            x = average_x / nr_stanga;
            y = average_x / nr_stanga;

            // x si y fata de centru
            x -= rows / 2;
            y -= cols / 2;
        }
        else
        {
            side = E_SIDE.RIGHT;

            // asta e pozitia obiectului dar pe noi ne intereseaza poz fata de centru pozei
            x = average_x / nr_dreapta;
            y = average_x / nr_dreapta;

            // x si y fata de centru
            x -= rows / 2;
            y -= cols / 2;
        }

        return input;
    }

    public E_SIDE getSide() { return side; } // obiectul este in stanga / dreapta sau NONE daca nu il detecteaza

    public double[] getXY() // pozitia fata de centrul pozei
    {
        return new double[]{x, y};
    }

    public enum E_SIDE {LEFT,RIGHT,NONE}
}