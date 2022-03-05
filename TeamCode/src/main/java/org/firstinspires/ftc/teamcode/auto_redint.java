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
@Autonomous(name = "AUTONOMOUS_redint")
public class auto_redint extends LinearOpMode {
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


        //----------------------------------------------------------------------------------------------

        //traiectorii redside intern
        Pose2d startPose= new Pose2d(0,-47.24,0);
        drive.setPoseEstimate(startPose);
        Trajectory f1 = drive.trajectoryBuilder(startPose)

                .splineTo(new Vector2d(22,-27),0)

                .build();

        Trajectory duck =drive.trajectoryBuilder(f1.end())
                //.splineTo(new Vector2d(5,25),90)
                .lineToSplineHeading(new Pose2d(7.7,25,Math.toRadians(90)))
                .build();
        Trajectory warehouse = drive.trajectoryBuilder(duck.end())
                .lineToSplineHeading(new Pose2d(-0.5,0,Math.toRadians(270)))

                .build();
        Trajectory streif=drive.trajectoryBuilder(warehouse.end())
                .strafeRight(2)
                .build();
        Trajectory f2 =drive.trajectoryBuilder(streif.end())
                .forward(85) // daca nu 90
                .addTemporalMarker(0.1,()->
                {   cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.7);

                })
                .build();

        Trajectory streiff = drive.trajectoryBuilder(f2.end())
                .strafeRight(1)
                .build();
        Trajectory ia_bila_cub = drive.trajectoryBuilder(streiff.end())
                .back(40)
                .build();

        Trajectory revers_card = drive.trajectoryBuilder(ia_bila_cub.end())
                .lineToLinearHeading(new Pose2d(24.5, -30, Math.toRadians(12)))
                .addTemporalMarker(0.1,()->
                {
                    cremaliera.setTargetPosition(-2900);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);


                })
                .build();

        Trajectory ionutz = drive.trajectoryBuilder(revers_card.end())
                .lineToLinearHeading(new Pose2d(0.4, -40, Math.toRadians(270)))
                .build();
        Trajectory streif2= drive.trajectoryBuilder(ionutz.end())
                .strafeRight(5.25)
                .build();

        Trajectory inapoi = drive.trajectoryBuilder(streif2.end())
                .forward(50)
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


        while (opModeIsActive())
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
            while(runtime2.time()<3.0)
            carusel.setPower(-0.4);
            drive.followTrajectory(warehouse);sleep(500);
            drive.followTrajectory(streif);
            carusel.setPower(0);
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
          //  sleep(1000);

            break;
        }

    }
}