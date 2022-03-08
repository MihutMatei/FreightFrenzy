package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
@Autonomous(name = "AUTONOMOUS_redext")
public class auto_redext extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    private DcMotorEx cremaliera;
    private DcMotorEx cascade;
    private DcMotorEx intake;
    private DcMotor carusel;
    private Servo cuva;
    private  Servo outake;

    //unfinished

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive =new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cremaliera =hardwareMap.get(DcMotorEx.class,"cremaliera");
        cascade =hardwareMap.get(DcMotorEx.class,"cascade");
        cuva =hardwareMap.get(Servo.class,"cuva");
        intake =hardwareMap.get(DcMotorEx.class,"intake");
        carusel=hardwareMap.get(DcMotor.class, "carusel");
        outake=hardwareMap.get(Servo.class,"outake");

        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        cascade.setDirection(DcMotorSimple.Direction.REVERSE);



        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runtime2 = new ElapsedTime(0);


        //----------------------------------------------------------------------------------------------

        //traiectorii redside extern
        Pose2d startPose= new Pose2d(0,0,0);
        Pose2d mypose=drive.getPoseEstimate();
        drive.setPoseEstimate(startPose);
        Trajectory f1 = drive.trajectoryBuilder(startPose)

                .lineToSplineHeading(new Pose2d(23,-25,Math.toRadians(350)))

                .build();


        Trajectory duck =drive.trajectoryBuilder(f1.end())
                .lineToSplineHeading(new Pose2d(7.7,25,Math.toRadians(90)))
                .build();


        Trajectory turn =drive.trajectoryBuilder(duck.end())
                .lineToSplineHeading(new Pose2d(8,23,Math.toRadians(180)))
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
        int zone = pipeline.getAverage();
        int avg1;
        int avg2;
        int avg3;


        while (opModeIsActive())
        {
            Trajectory shipp=drive.trajectoryBuilder(mypose)
                .lineToSplineHeading(new Pose2d(23,-25,Math.toRadians(350)))
                .addTemporalMarker(0.1,()->
                {
                    cascade.setTargetPosition(-700);
                    cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.5);
                    outake.setPosition(0.5);
                })
                .build();
            
            telemetry.update();
            drive.followTrajectory(f1);
            sleep(500);
            telemetry.clear();
            telemetry.update();
            telemetry.addData("Zona", zone);
            telemetry.update();
            if(zone==1||zone==0)
            {

                cascade.setTargetPosition(-200);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.9);
                while (cascade.isBusy())
                {
                    outake.setPosition(0.5);
                }
              //cuva
                cascade.setTargetPosition(0);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.9);


            }
            if(zone==2)
            {
                cascade.setTargetPosition(-600);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.9);
                while (cascade.isBusy())
                {
                    outake.setPosition(0.5);

                }

               //cuva
              cascade.setTargetPosition(0);
              cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              cascade.setPower(0.4);


            }
            if(zone==3)
            {
                    cascade.setTargetPosition(-700);
                    cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.9);
                    sleep(500);
                    while(cascade.isBusy())
                    {
                        outake.setPosition(0.5);

                    }
                  //cuva
                    cascade.setTargetPosition(0);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.9);
            }

            outake.setPosition(0);

            drive.followTrajectory(duck);
            cascade.setVelocity(0);
            runtime2.reset();
            while(runtime2.time()<3.0)
            carusel.setPower(-0.4);
            drive.followTrajectory(turn);
            avg1=pipeline.getAverage1();
            avg2=pipeline.getAverage2();
            avg3=pipeline.getAverage3();
            if(avg1<120)
            {
                //traj
            }
           else  if(avg2<120)
            {
                //traj
            }
           else  if(avg3<120)
            {
                //traj

            }
           mypose=drive.getPoseEstimate();
           drive.followTrajectory(shipp);
           //cuva
            //drive.followTrajectory(park)





            break;
        }

    }
}
