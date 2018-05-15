package org.firstinspires.ftc.teamcode;
/**
 * Recreated by nerdxoverboard on 11/17/2017.
 */



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.teamcode.HardwarePushbot.MOTOR_TICKS;

// (c) 2017 - FTC Team 11242 - Error 451 - Ferris High School - Ferris, TX
@Autonomous(name = "Red: Two", group = "Concept")
//@Disabled <-- Keep commented unless this Opmode is not in use
public class AutonomousJVer2 extends LinearOpMode {


    DcMotor leftMotor,  // left drive wheel
            rightMotor, // right drive wheel
            leftTred,   // left tread
            rightTred;  // right tread

    ColorSensor colorSensor;

    Servo armShoulder,
            armElbow;  // shoulder servo on robot

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor   = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor  = hardwareMap.get(DcMotor.class, "right_drive");
        leftTred    = hardwareMap.get(DcMotor.class, "left_tred");
        rightTred   = hardwareMap.get(DcMotor.class, "right_tred");
        armShoulder = hardwareMap.get(Servo.class, "shoulder");
        armElbow    = hardwareMap.get(Servo.class, "elbow");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        waitForStart();

        telemetry.addData("Status", "Driving");
        telemetry.update();

        // Beginning Part 1 of Autonomous Mode

        while (opModeIsActive()) {

            armElbow.setPosition(0.5); //0 is right side
            sleep(250);
            armShoulder.setPosition(0.75);
            sleep(250);
            armElbow.setPosition(1);
            sleep(250);
            armShoulder.setPosition(0.52); //0 is right side
            sleep(350);
            armShoulder.setPosition(0.37); //0 is right side
            sleep(550);
            armShoulder.setPosition(0.22); //0 is right side
            sleep(800);
/*
            colorSensor.enableLed(true);

            if (colorSensor.blue() > colorSensor.red()){
                while(leftMotor.getCurrentPosition() != -MOTOR_TICKS) {
                    // Add motor ticks for total
                    total_ticks += MOTOR_TICKS;

                    // Set Target Position to 1 revolution
                    leftLift.setTargetPosition(-MOTOR_TICKS);
                    rightLift.setTargetPosition(MOTOR_TICKS);

                    leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Set Power
                    leftLift.setPower(-1);
                    rightLift.setPower(1);
            }
            */



            armShoulder.setPosition(0.12); //0 is right side
            sleep(1100);
            armElbow.setPosition(0.75);
            sleep(2500);


            /*leftMotor.setPower(-0.25); // drive forward at quarter speed
            rightMotor.setPower(0.25); // drive forward at quarter speed
            sleep(1250); // stop after one second
            leftMotor.setPower(0); // stop left motor
            rightMotor.setPower(0); // stop right motor

            leftTred.setPower(-0.25); //  treads move up at quarter speed
            rightTred.setPower(0.25); // treads move up at quarter speed
            sleep(1000); // stop after one second
            leftTred.setPower(0); // stop left tread
            rightTred.setPower(0); //stop right tread*/

           /* sleep(5000);
            armElbow.setPosition(1); //0 is right side
            armShoulder.setPosition(0.5); //0 is right side
            armShoulder.setPosition(1);
            sleep(150);
            armElbow.setPosition(0);
            sleep(25000);*/
        }
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
