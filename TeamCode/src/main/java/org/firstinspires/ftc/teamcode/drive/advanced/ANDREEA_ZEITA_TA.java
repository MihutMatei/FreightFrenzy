package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
@Disabled
public class ANDREEA_ZEITA_TA extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor intake;
    private DcMotor roata;
    private DcMotor cremaliera;
    private DcMotor extindere;
    private Servo cuva;


    private Servo INTAKE_ROTATIE;
    private Servo INTAKE_MISCARE;


    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        roata = hardwareMap.get(DcMotorEx.class, "roata");
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        cremaliera = hardwareMap.get(DcMotorEx.class, "cremaliera");
//        extindere = hardwareMap.get(DcMotorEx.class, "extindere");
//        cuva = hardwareMap.get(Servo.class,"cuva");


//        INTAKE_MISCARE = hardwareMap.get(Servo.class, "A");
//        INTAKE_ROTATIE = hardwareMap.get(Servo.class, "B");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive()){

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    ));

                        drive.update();
                Pose2d poseEstimate = drive.getPoseEstimate();
                if(gamepad1.left_bumper)roata.setPower(0.5);
                if(gamepad1.right_bumper)roata.setPower(0);

//                if(gamepad2.dpad_up)cremaliera.setPower(-0.6);
//                if(gamepad2.dpad_down)cremaliera.setPower(0.6);
//                if(gamepad2.dpad_left)cremaliera.setPower(0);
//                if(gamepad1.dpad_left)extindere.setPower(0.4);
//                if(gamepad1.dpad_right)extindere.setPower(-0.4);
//                if(gamepad1.dpad_left)extindere.setPower(0);
//                if(gamepad1.a)intake.setPower(0.5);
//                if(gamepad1.b)intake.setPower(-0.5);
//                if(gamepad1.y)intake.setPower(0);
//                if(gamepad1.x)cuva.setPosition(0.4);
//



            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            telemetry.update();
        }
    }

}
