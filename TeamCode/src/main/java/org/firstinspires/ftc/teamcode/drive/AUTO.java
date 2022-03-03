package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@Autonomous(name="AUTO",group = "Vineri")
public class AUTO extends LinearOpMode {

    private DcMotor cremaliera;
    private DcMotor cascade;
    private Servo intake_servo;

    private DcMotor carusel;

    @Override
    public void runOpMode() {

        cremaliera =hardwareMap.get(DcMotor.class,"cremaliera");
        cascade =hardwareMap.get(DcMotor.class,"cascade");
        intake_servo =hardwareMap.get(Servo.class,"intake_servo");
       // carusel=hardwareMap.get(DcMotor.class, "carusel");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0,0);

        drive.setPoseEstimate(startPose);

        // TODO: traiectorii forward

        Trajectory  traj1 = drive.trajectoryBuilder(new Pose2d())
        .lineToSplineHeading(new Pose2d(40,75,Math.toRadians(-90)))
                .build();

        Trajectory traj2 =drive.trajectoryBuilder(traj1.end())
                .forward(8)
        .splineTo(new Vector2d(90,45),-45)
               .splineTo(new Vector2d(80,10),0)
                .forward(30)

                .build();

        Trajectory traj3 =drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(143,35,Math.toRadians(90)))

                .build();

       Trajectory traj4 =drive.trajectoryBuilder(traj3.end())
               .forward(15)

                .build();
        Trajectory traj5 =drive.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(119,75),Math.toRadians(90))

                .build();
        Trajectory traj6 =drive.trajectoryBuilder(traj5.end())
                .forward(10)
               // .lineToSplineHeading(new Pose2d(119,85,Math.toRadians(0)))
               // .forward(20)
                .build();
      //  Pose2d lastpose = new Pose2d(119, 85,90);
        Trajectory traj7 =drive.trajectoryBuilder(traj6.end())
               .strafeRight(25)
               // .lineTo(new Vector2d(119,85))
                // .lineToSplineHeading(new Pose2d(119,85,Math.toRadians(0)))
                // .forward(20)
                .build();

        //wait for the game to begin
        intake_servo.setPosition(0.1);


        waitForStart();

        if(opModeIsActive())
        {
            while(opModeIsActive())
            {
                drive.followTrajectory(traj1);
                sleep(300);

                drive.followTrajectory(traj2);
                sleep(300);
                drive.followTrajectory(traj3);
                drive.followTrajectory(traj4);
                sleep(500);
                drive.followTrajectory(traj5);
                drive.followTrajectory(traj6);
            //    drive.turn(Math.toRadians(-90));
                sleep(1000);
                drive.followTrajectory(traj7);


                break;
            }
        }

    }
}
