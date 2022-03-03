package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
@Disabled
public class IONUTZ extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor A;
    private DcMotor B;
    private DcMotor C;

    boolean pressedA = true;
    boolean pressedB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");


        A = hardwareMap.get(DcMotorEx.class, "A");
        B = hardwareMap.get(DcMotorEx.class, "B");
        C = hardwareMap.get(DcMotorEx.class, "C");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Delay extindere telescopica
        //C.setPower(-1);
        //sleep(4500);
        //C.setPower(0);
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

            //

            if(gamepad2.dpad_up && pressedA)
            {
                A.setPower(1);
                B.setPower(-1);
                sleep(800);
                A.setPower(0);
                B.setPower(0);
                pressedA = false;
                pressedB = true;
            }
            if(gamepad2.dpad_down && pressedB)
            {
                A.setPower(-1);
                B.setPower(1);
                sleep(800);
                A.setPower(0);
                B.setPower(0);
                pressedA = true;
                pressedB = false;
            }
            if(gamepad2.dpad_left)
            {
                C.setPower(-1);
                sleep(4350);
                C.setPower(0);
            }
            if(gamepad2.dpad_right)
            {
                C.setPower(1);
                sleep(4450);
                C.setPower(0);
            }
//            if(gamepad2.left_stick_x > 0.1)
//            {
//                C.setPower(gamepad2.left_stick_x);
//            }
//            if(gamepad2.left_stick_x < -0.1)
//            {
//                C.setPower(gamepad2.left_stick_x);
//            }




            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            telemetry.update();
        }
    }

}
