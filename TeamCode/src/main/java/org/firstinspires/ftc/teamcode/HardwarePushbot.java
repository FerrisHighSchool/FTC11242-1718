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

package org.firstinspires.ftc.teamcode;
/**
 * Created by nerdxoverboard on 9/21/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left drive motor:        "left_drive"
 * Motor channel:  Right drive motor:       "right_drive"
 * Motor channel:  Left drive lift:         "left_lift"
 * Motor channel:  Right drive lift:        "right_lift"
 * Motor channel:  Left tread:              "left_tred"
 * Motor channel:  Right tread:             "right_tred"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  leftMotor = null;
    public DcMotor  leftLift = null;
    public DcMotor  rightMotor = null;
    public DcMotor  leftTred = null;
    public DcMotor  rightTred = null;
    public DcMotor  rightLift = null;
    /*public Servo  leftClaw    = null;
    public Servo    rightClaw   = null; */

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    //variables used for lift encoders
    public static final double COUNTS_PER_MOTOR_REV = 1120; // pulses per rotation on AndyMark NeveRest 40 Gearmotor Encoder
    public static final double WHEEL_DIAMETER_CM = 3.1; // for finding circumference
    public static final double COUNTS_PER_CM =  COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_CM * Math.PI); // about 115 pulses per cm


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor = hwMap.get(DcMotor.class, "left_drive");
        rightMotor = hwMap.get(DcMotor.class, "right_drive");
        leftTred = hwMap.get(DcMotor.class, "left_tred");
        rightTred = hwMap.get(DcMotor.class, "right_tred");
        leftLift = hwMap.get(DcMotor.class, "left_lift");
        rightLift = hwMap.get(DcMotor.class, "right_lift");

        // Set Direction for Motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftTred.setDirection(DcMotor.Direction.REVERSE); // set at REVERSE to go forward, since they are AndyMark motors, true of all three right motors
        rightTred.setDirection(DcMotor.Direction.FORWARD); // set at FORWARD, since they are AndyMark motors, true of all three left motors


        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftTred.setPower(0);
        rightTred.setPower(0);
        leftMotor.setPower(0);
        rightLift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // change to encoders later
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //change to encoders later

        leftTred.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightTred.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Define and initialize ALL installed servos.
        /* leftClaw  = hwMap.get(Servo.class, "left_hand");
        rightClaw = hwMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO); */
    }
}

