package org.firstinspires.ftc.teamcode;
/**
 * Recreated by nerdxoverboard on 11/17/2017.
 */



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.I2cDevice;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import com.qualcomm.robotcore.hardware.TouchSensor;



// (c) 2017 - FTC Team 11242 - Error 451 - Ferris High School - Ferris, TX



@Autonomous(name = "Red: A", group = "Concept")

//@Disabled <-- Keep commented unless this Opmode is not in use



public class CenterRaw extends LinearOpMode {


    DcMotor leftMotor,  // left drive wheel
            rightMotor, // right drive wheel
            leftTred,   // left tread
            rightTred;  // right tread

    static final double COUNTS_PER_MOTOR_REV = 1120;  // For NeveRest 40 Gearmotor Encoder

    static final double WHEEL_DIAMETER_INCHES = 4.25;  // For figuring circumference

    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV /

            (WHEEL_DIAMETER_INCHES * 3.1415); // About 83.89 ticks per inch


    @Override

    public void runOpMode() throws InterruptedException {


        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftTred = hardwareMap.get(DcMotor.class, "left_tred");
        rightTred = hardwareMap.get(DcMotor.class, "right_tred");

        waitForStart();


        telemetry.addData("Status", "Driving");

        telemetry.update();


        // Beginning Part 1 of Autonomous Mode

      /*  drive(1500);
        stopp(1000);
        turn(1100);
        driveslow(1250);
        stopp(1500);
        release(1500);
        reverse(2000);
        drive(1500);
        stop();
    */
      drive(750);
      stopp(5000);//good
      drive(1000);
      turn(1100);
      /*driveslow(250);
      release(1000);
      reverse(1000);
      drive(500);
      */
      stop();

    }



    public void stopp(int time) throws InterruptedException {
        if (opModeIsActive()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(time);

        }
    }

    public void turn(int time) throws InterruptedException {
        if (opModeIsActive()) {
            leftMotor.setPower(-.25);
            rightMotor.setPower(-.25);
            sleep(time);

        }
    }


    public void drive(int time) throws InterruptedException {

        int newRightTarget;
        if (opModeIsActive()) {
            leftMotor.setPower(-.25);
            rightMotor.setPower(.25);
            sleep(time);

        }

    }
    public void driveslow(int time) throws InterruptedException {

        int newRightTarget;
        if (opModeIsActive()) {
            leftMotor.setPower(-.25/2);
            rightMotor.setPower(.25/2);
            sleep(time);

        }

    }

    public void reverse(int time) throws InterruptedException {
        if (opModeIsActive()) {
            leftMotor.setPower(.25);
            rightMotor.setPower(-.25);
            sleep(time);

        }

    }

    public void release(int time) throws InterruptedException {
        if (opModeIsActive()) {
            leftTred.setPower(-.25);
            rightTred.setPower(.25);
            sleep(time);

        }
    }
}
