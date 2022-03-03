 /* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Test_drive", group="Linear Opmode")

public class Test_drive extends LinearOpMode {


    private DcMotorEx cremaliera;
    private DcMotorEx cascade;
    private DcMotorEx intake;
    private DcMotor carusel;
//    private CRServo ruleta;
//    private CRServo ruleta_x;
//    private CRServo ruleta_z;


    @Override
    public void runOpMode() throws InterruptedException {


        cremaliera =hardwareMap.get(DcMotorEx.class,"cremaliera");
        cascade =hardwareMap.get(DcMotorEx.class,"cascade");
        intake =hardwareMap.get(DcMotorEx.class,"intake");
        carusel=hardwareMap.get(DcMotor.class, "carusel");
      //  ruleta =hardwareMap.get(CRServo.class,"ruleta");
       // ruleta_x =hardwareMap.get(CRServo.class,"ruleta_x");
       // ruleta_z =hardwareMap.get(CRServo.class,"ruleta_z");
        int power =3;

        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        cascade.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       //    cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ruleta_x.setPower(0);
        //ruleta.setPower(0);
        //ruleta_z.setPower(0);
        waitForStart();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        while (opModeIsActive()) {

            ElapsedTime runtime1 = new ElapsedTime(0);


            //drive.setMotorPowers(1,1,1,1);
            drive.setWeightedDrivePower(
                    new Pose2d(
                           -gamepad1.left_stick_y/2,
                           -gamepad1.left_stick_x /2,
                           -gamepad1.right_stick_x/2
                    )

            );
            drive.update();






            cremaliera.setTargetPosition(-3000);


        if(gamepad2.left_stick_y>0) {
            cremaliera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          cremaliera.setPower(0.75);
        }
        else if(gamepad2.left_stick_y<0) {
            cremaliera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            cremaliera.setPower(-0.75);
        }
        else {
            cremaliera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            cremaliera.setPower(0);
        }
        //TODO:Limitare motor cascade varianta cu encoder si elapsed time
        if(gamepad2.right_stick_y>0) {
            cascade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cascade.setPower(0.50);
        }

        else if(gamepad2.right_stick_y<0) {
            cascade.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cascade.setPower(-0.50);
        }
        else cascade.setPower(0);
        //TODO:cod pentru utiliza
        if(gamepad2.dpad_left) {
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.7);

        }
        if(gamepad2.dpad_right) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(1);

        }
        if(gamepad2.dpad_down) {
            intake.setPower(0);
        }

        if(gamepad2.cross) {
            carusel.setPower(-0.5);
        }
        if(gamepad2.circle) {
            carusel.setPower(0);
        }
/**
 * controale pentru ruleta:
 * */
//        if(gamepad1.dpad_right)
//        {
//                ruleta_x.setDirection(DcMotorSimple.Direction.FORWARD);
//                ruleta_x.setPower(0.1);
//
//        }
//        if(gamepad1.dpad_left)
//        {
//                ruleta_x.setDirection(DcMotorSimple.Direction.REVERSE);
//                ruleta_x.setPower(0.1);
//
//        }
//
//        if(gamepad1.dpad_down)
//        {
//                ruleta_z.setDirection(DcMotorSimple.Direction.FORWARD);
//                ruleta_z.setPower(0.1);
//
//        }
//        if(gamepad1.dpad_up)
//        {
//                ruleta_z.setDirection(DcMotorSimple.Direction.REVERSE);
//                ruleta_z.setPower(0.1);
//
//        }
//        if(gamepad1.square)
//        {
//                ruleta.setDirection(DcMotorSimple.Direction.FORWARD);
//                ruleta.setPower(1.0);
//
//        }
//        if(gamepad1.circle)
//        {
//                ruleta.setDirection(DcMotorSimple.Direction.REVERSE);
//                ruleta.setPower(1.0);
//
//        }
//        if(gamepad1.triangle) {
//                ruleta.setPower(0);
//                ruleta_x.setPower(0);
//                ruleta_z.setPower(0);
//        }
        /*if(gamepad2.left_bumper)
        {   cremaliera.setTargetPosition(-2700);
            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setPower(0.75);
            while(cremaliera.isBusy())
            {

            }
            cascade.setTargetPosition(650);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
            while(cascade.isBusy())
            {

            }
        }
        if(gamepad2.right_bumper)
        {
            cascade.setTargetPosition(0);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);

            cremaliera.setTargetPosition(-300);
            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setVelocity(3000);

        }*/

          //  else ruleta.setPower(0);
          telemetry.addData("cascade",cascade.getCurrentPosition());
            telemetry.addData("cremaliera",cremaliera.getCurrentPosition());
      telemetry.update();

/**
 * TODO:
 *          - gasit servo programer pentru a face tote servourile de la ruleta continuous rotation
 *          - sa nu mai plang :(
 */



        }
    }
}
