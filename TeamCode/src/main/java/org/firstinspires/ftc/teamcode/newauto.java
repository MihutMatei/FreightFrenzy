package org.firstinspires.ftc.teamcode;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@Autonomous(name = "NewAuto")
public class newauto extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    private DcMotorEx cremaliera;
    private DcMotorEx cascade;
    private Servo intake_servo;
    private DcMotorEx intake;
    private DcMotor carusel;
    private CRServo ruleta;
    private CRServo ruleta_x;
    private CRServo ruleta_z;
    static int zona=0;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive =new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cremaliera =hardwareMap.get(DcMotorEx.class,"cremaliera");
        cascade =hardwareMap.get(DcMotorEx.class,"cascade");
/*
        intake_servo =hardwareMap.get(Servo.class,"intake_servo");
*/
        intake =hardwareMap.get(DcMotorEx.class,"intake");
        carusel=hardwareMap.get(DcMotor.class, "carusel");
        //ruleta =hardwareMap.get(CRServo.class,"ruleta");
        // ruleta_x =hardwareMap.get(CRServo.class,"ruleta_x");
        //   ruleta_z =hardwareMap.get(CRServo.class,"ruleta_z");


        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        cascade.setDirection(DcMotorSimple.Direction.REVERSE);



        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runtime1 = new ElapsedTime(0);
        ElapsedTime runtime2 = new ElapsedTime(0);
        ElapsedTime runtime3 = new ElapsedTime(0);
        ElapsedTime runtime4 = new ElapsedTime(0);

        // TODO: sa inmultim valoriile coordonatelor din traiectorii cu coef "static double coef = 1.394230769230769;" e raport dintre roboti la sasiu
        Pose2d startPose= new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);
        Trajectory f1 = drive.trajectoryBuilder(startPose)

                .splineTo(new Vector2d(22.2,-25),0)

                .build();

        Trajectory warehouse = drive.trajectoryBuilder(f1.end())
                .lineToSplineHeading(new Pose2d(-0.5,-40,Math.toRadians(270)))
                .addTemporalMarker(0.1,()->
                {   intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.6);
                    cascade.setTargetPosition(-20);
                    /* cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.4);
                    cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                })
                .build();

        Trajectory streif=drive.trajectoryBuilder(warehouse.end())
                .strafeRight(5)
                .build();
        Trajectory f2 =drive.trajectoryBuilder(streif.end())
                .forward(55) // daca nu 90

                .build();


        Trajectory streiff = drive.trajectoryBuilder(f2.end())
                .strafeRight(4)
                .build();
        Trajectory ia_bila_cub = drive.trajectoryBuilder(streiff.end())
                .back(40)
                .build();

        Trajectory revers_card = drive.trajectoryBuilder(ia_bila_cub.end())
                .lineToLinearHeading(new Pose2d(23, -35, Math.toRadians(10)))
                .addTemporalMarker(0.1,()->
                {
                    cremaliera.setTargetPosition(-2900);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);


                })
                .build();
        Trajectory warehouse2 =drive.trajectoryBuilder(revers_card.end())
                .lineToSplineHeading(new Pose2d(-1.5,-40,Math.toRadians(270)))
                .addTemporalMarker(0.1,()->
                {   intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.6);
                    cascade.setTargetPosition(-20);
                    /* cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.4);
                    cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                })

                .build();
        Trajectory streif2=drive.trajectoryBuilder(warehouse2.end())
                .strafeRight(5)
                .build();
        Trajectory f22 =drive.trajectoryBuilder(streif2.end())
                .forward(60) // daca nu 90

                .build();


        Trajectory streiff2 = drive.trajectoryBuilder(f22.end())
                .strafeRight(4)
                .build();
        Trajectory ia_bila_cub2 = drive.trajectoryBuilder(streiff2.end())
                .back(40)
                .build();

        Trajectory revers_card2 = drive.trajectoryBuilder(ia_bila_cub2.end())
                .lineToLinearHeading(new Pose2d(24, -35, Math.toRadians(10)))
                .addTemporalMarker(0.1,()->
                {
                    cremaliera.setTargetPosition(-2900);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);


                })
                .build();
        Trajectory warehouse3=drive.trajectoryBuilder(revers_card2.end())
                .lineToSplineHeading(new Pose2d(-2.5,-40,Math.toRadians(270)))
                .addTemporalMarker(0.1,()->
                {   intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.6);
                    cascade.setTargetPosition(-20);
                    /* cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.4);
                    cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                })

                .build();
        Trajectory streif3=drive.trajectoryBuilder(warehouse3.end())
                .strafeRight(5)
                .build();
        Trajectory f23 =drive.trajectoryBuilder(streif3.end())
                .forward(65) // daca nu 90

                .build();


        Trajectory streiff3 = drive.trajectoryBuilder(f23.end())
                .strafeRight(4)
                .build();
        Trajectory ia_bila_cub3 = drive.trajectoryBuilder(streiff3.end())
                .back(40)
                .build();

        Trajectory revers_card3 = drive.trajectoryBuilder(ia_bila_cub3.end())
                .lineToLinearHeading(new Pose2d(24, -35, Math.toRadians(10)))
                .addTemporalMarker(0.1,()->
                {
                    cremaliera.setTargetPosition(-2900);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);


                })
                .build();

        Trajectory warehouse4 =drive.trajectoryBuilder(revers_card3.end())
                .lineToSplineHeading(new Pose2d(-3.5,-40,Math.toRadians(270)))
                .addTemporalMarker(0.1,()->
                {   intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.6);
                    cascade.setTargetPosition(-20);
                    /* cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.4);
                    cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                })

                .build();
        Trajectory streif4=drive.trajectoryBuilder(warehouse4.end())
                .strafeRight(5)
                .build();
        Trajectory f24 =drive.trajectoryBuilder(streif4.end())
                .forward(70) // daca nu 90

                .build();






        //----------------------------------------------------------------------------------------------

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        while (true) {
            telemetry.addData("Type", pipeline.getType());
            telemetry.addData("Zona", pipeline.getAverage());
            telemetry.addData("Averagefin",pipeline.getAveragefin());
            telemetry.addData("Average1",pipeline.getAverage1() );
            telemetry.addData("Average2",pipeline.getAverage2() );
            telemetry.addData("Average3",pipeline.getAverage3() );

            telemetry.update();
            sleep(0);
            if(isStopRequested())break;
            if(opModeIsActive())
                break;

        }
        int zone=zona;


        while (opModeIsActive())
        {  // ruleta.setPower(0);
//            ruleta_x.setPower(0);
//            ruleta_z.setPower(0);
            telemetry.update();
            drive.followTrajectory(f1);
           // sleep(500);
            telemetry.update();
            if(zone==1||zone==0)
            {   cremaliera.setTargetPosition(-1700);
                cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cremaliera.setVelocity(3000);
                while(cremaliera.isBusy())
                {

                }
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.6);
                sleep(500);

            }
            if(zone==2)
            {   intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(0.4);
                cremaliera.setTargetPosition(-2300);
                cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cremaliera.setVelocity(3000);
                while(cremaliera.isBusy()){

                }
             //  sleep(500);
                cascade.setTargetPosition(-600);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.4);

               sleep(500);
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.35);
               sleep(500);
                cascade.setTargetPosition(0);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.4);


            }
            if(zone==3)
            {       intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(0.4);
                cremaliera.setTargetPosition(-3300);
                cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cremaliera.setVelocity(3000);
                while(cremaliera.isBusy()){

                }
              //  sleep(500);
                cascade.setTargetPosition(-650);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.4);
                sleep(500);
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.6);
              sleep(500);
               cascade.setTargetPosition(0);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.4);
            }
          //  sleep(1000) ;
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(0);
            drive.followTrajectory(warehouse);
           // sleep(500);
           // drive.followTrajectory(streif);
           // sleep(500);
            drive.followTrajectory(f2);
            sleep(500);
            drive.followTrajectory(streiff);
           // sleep(500);
            drive.followTrajectory(ia_bila_cub);
           // sleep(500);
            drive.followTrajectory(revers_card);
           // sleep(500);
            cascade.setTargetPosition(-600);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
            while(cascade.isBusy())

            {

            }
            sleep(200);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.6);
            sleep(500);
            drive.followTrajectory(warehouse2);

            drive.followTrajectory(streif2);
            //sleep(500);
            drive.followTrajectory(f22);
          //  sleep(500);
            drive.followTrajectory(streiff2);
          //  sleep(500);
            drive.followTrajectory(ia_bila_cub2);
           // sleep(500);
            drive.followTrajectory(revers_card2);
          //  sleep(500);
            cascade.setTargetPosition(-600);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
            while(cascade.isBusy())

            {

            }
            sleep(200);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.6);
            sleep(500);
            drive.followTrajectory(warehouse3);

            drive.followTrajectory(streif3);
           // sleep(500);
            drive.followTrajectory(f23);

            drive.followTrajectory(streiff3);
           // sleep(500);
            drive.followTrajectory(ia_bila_cub3);
           // sleep(500);
            drive.followTrajectory(revers_card3);
            cascade.setTargetPosition(-600);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
            while(cascade.isBusy())

            {

            }
            //sleep(1000);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.6);
            //sleep(1000);
            drive.followTrajectory(warehouse4);

            drive.followTrajectory(streif4);
           // sleep(500);
            drive.followTrajectory(f24);
          //  sleep(500);
            /*sleep(2500);

            drive.followTrajectory(duck);
            cascade.setVelocity(0);
            runtime2.reset();
            while(runtime2.time()<3.0)
                carusel.setPower(-0.4);
            drive.followTrajectory(warehouse);sleep(500);
            drive.followTrajectory(streif);
            drive.followTrajectory(f2);
            sleep(500);
            drive.followTrajectory(streiff);
            drive.followTrajectory(ia_bila_cub);
            sleep(500);
            drive.followTrajectory(revers_card);
            cascade.setTargetPosition(-600);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
            while(cascade.isBusy())

            {

            }
            sleep(1000);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.6);
            sleep(1000);
            drive.followTrajectory(ionutz);
            drive.followTrajectory(streif2);
            drive.followTrajectory(inapoi);
            sleep(1000);*/

            break;
        }

    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        private static final int THRESHOLD = 112;



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
}