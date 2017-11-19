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



@Autonomous(name = "Red: One", group = "Autonomous")

//@Disabled <-- Keep commented unless this Opmode is not in use



public class AutonomousJ extends LinearOpMode {


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


        leftMotor  = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftTred = hardwareMap.get(DcMotor.class, "left_tred");
        rightTred = hardwareMap.get(DcMotor.class, "right_tred");

        waitForStart();


        telemetry.addData("Status", "Driving");

        telemetry.update();


        // Beginning Part 1 of Autonomous Mode

        leftMotor.setPower(-0.25); // drive forward at quarter speed, negated to spin correct direction.
        rightMotor.setPower(0.25); // drive forward at quarter speed.
        sleep(1250); // sleep after 1.25 seconds.
        leftMotor.setPower(0); // kill power to left motor.
        rightMotor.setPower(0); // kill power to right motor.

        // Part 2 of Autonomous Mode

        leftTred.setPower(-0.25); // move up at quarter speed, negated to spin correct direction.
        rightTred.setPower(0.25); // move up at quarter speed.
        sleep(1000); // sleep after 1 second.
        leftTred.setPower(0); // kill power to left tread.
        rightTred.setPower(0); // kill power to right tread.
    }


    public void turnAt(double angle, double speed) throws InterruptedException {


        leftMotor.setPower(0);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        sleep(250);


    }


    public void drive() throws InterruptedException {

        int newRightTarget;


        // Ensure that the opmode is still active

        if (opModeIsActive()) {


            leftMotor.setPower(1);

            rightMotor.setPower(1);


            // Turn off RUN_TO_POSITION

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);

        }

    }
}
