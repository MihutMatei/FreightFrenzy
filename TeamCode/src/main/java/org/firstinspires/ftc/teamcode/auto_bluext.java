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

@Autonomous(name = "AUTONOMOUS_bluext")
public class auto_bluext extends LinearOpMode {
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
    //unfinished

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive =new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cremaliera =hardwareMap.get(DcMotorEx.class,"cremaliera");
        cascade =hardwareMap.get(DcMotorEx.class,"cascade");
        intake =hardwareMap.get(DcMotorEx.class,"intake");
        carusel=hardwareMap.get(DcMotor.class, "carusel");



        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        cascade.setDirection(DcMotorSimple.Direction.REVERSE);



        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runtime1 = new ElapsedTime(0);
        ElapsedTime runtime2 = new ElapsedTime(0);
        ElapsedTime runtime3 = new ElapsedTime(0);
        ElapsedTime runtime4 = new ElapsedTime(0);


        //----------------------------------------------------------------------------------------------

        //traiectorii blueside extern
        Pose2d startPose= new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);
        Trajectory f1 = drive.trajectoryBuilder(startPose)

              //  .splineTo(new Vector2d(24,18),0)
                .lineToSplineHeading(new Pose2d(23.5,17.5,Math.toRadians(10)))

                .build();


        Trajectory duck =drive.trajectoryBuilder(f1.end())
                .lineToSplineHeading(new Pose2d(0.5,-31,Math.toRadians(135)))
                .build();
        Trajectory parked =drive.trajectoryBuilder(duck.end())
                .lineToSplineHeading(new Pose2d(20,-32,Math.toRadians(90)))
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


        if (opModeIsActive())
        {  // ruleta.setPower(0);
//            ruleta_x.setPower(0);
//            ruleta_z.setPower(0);
            telemetry.update();
            drive.followTrajectory(f1);
            sleep(500);
            telemetry.clear();
            telemetry.update();
            telemetry.addData("Zona", zone);
            telemetry.update();
            if(zone==1||zone==0)
            {   cremaliera.setTargetPosition(-1400);
                cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cremaliera.setVelocity(3000);
                while(cremaliera.isBusy())
                {

                }


                cascade.setTargetPosition(-200);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.4);
                while (cascade.isBusy())
                {

                }
                sleep(500);
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.6);
                sleep(1000);
                cascade.setTargetPosition(0);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.4);


            }
            if(zone==2)
            {  // intake.setDirection(DcMotorSimple.Direction.REVERSE);
                //intake.setPower(0.4);
                cremaliera.setTargetPosition(-2300);
                cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cremaliera.setVelocity(3000);
                while(cremaliera.isBusy()){

                }
                sleep(500);
                cascade.setTargetPosition(-600);
                cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cascade.setPower(0.4);
                while (cascade.isBusy())
                {

                }

                sleep(500);
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setPower(0.35);
                sleep(1000);
              cascade.setTargetPosition(0);
              cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              cascade.setPower(0.4);


            }
            if(zone==3)
            {      // intake.setDirection(DcMotorSimple.Direction.REVERSE);
                   // intake.setPower(0.4);
                    cremaliera.setTargetPosition(-3000);
                    cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                    while(cremaliera.isBusy()){

                    }
                    sleep(500);
                    cascade.setTargetPosition(-700);
                    cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.4);
                    sleep(500);
                    while(cascade.isBusy())
                    {

                    }
                    sleep(500);
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.7);
                    sleep(1000);
                    cascade.setTargetPosition(0);
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.4);
            }


            sleep(500);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(0);
            drive.followTrajectory(duck);
            cascade.setVelocity(0);
            runtime2.reset();
            while(runtime2.time()<3.0) {
                if(runtime2.time()<1.5)
                carusel.setPower(0.4);
                else  carusel.setPower(0.7);
            }

            drive.followTrajectory(parked);


        }
    }
}