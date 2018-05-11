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
 * Created by nerdxoverboard on 2/1/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.*;

import static org.firstinspires.ftc.teamcode.HardwarePushbot.MOTOR_TICKS;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *1`34
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Lift", group="Linear Opmode")

public class Lift extends LinearOpMode {

    //private OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null; //AndyMark NeveRest 40
    private DcMotor rightMotor = null; //AndyMark NeveRest 40
    private DcMotor leftTred = null; //PITSCO TETRIX Motor
    private DcMotor rightTred = null; //PITSCO TETRIX Motor
    private DcMotor leftLift = null; //AndyMark NeveRest 40
    private DcMotor rightLift = null; //AndyMark NeveRest 40


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftTred = hardwareMap.get(DcMotor.class, "left_tred");
        rightTred = hardwareMap.get(DcMotor.class, "right_tred");
        leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift = hardwareMap.get(DcMotor.class, "right_lift");

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            boolean treadsUp;
            boolean treadsDown;
            boolean liftTwo;
            boolean lowerAll;

            double drive = -gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_y;
            liftTwo = gamepad1.a;
            lowerAll = gamepad1.b;
            treadsUp = gamepad1.dpad_up;
            treadsDown = gamepad1.dpad_down;

            // Button A is pressed
            if (liftTwo){
                    while(leftLift.getCurrentPosition() != -MOTOR_TICKS) {
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
                // Let while statement finish
                idle();
                // Set both motor power to 0
                leftLift.setPower(0);
                rightLift.setPower(0);

                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Reset Encoders for every time button is pressed
                leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Button B is pressed
            if(lowerAll) {
                // Function lowers leftLift and rightLift motors
                lowerAll(total_ticks);
            }

            //boolean for tread rotations
            if(treadsDown){
                leftTred.setPower(-1);
                rightTred.setPower(1);
            }
            else if(treadsUp){
                leftTred.setPower(1);
                rightTred.setPower(-1);
            } else {
                leftTred.setPower(0);
                rightTred.setPower(0);
            }
            leftPower = Range.clip(drive + turn, -1.0, 1.0);//gage drive
            rightPower = Range.clip(drive - turn, -1.0, 1.0);//gage drive

            leftMotor.setPower(leftPower);  //motor power
            rightMotor.setPower(rightPower);  //motor power

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Lift Position", "left (%d), right (%d)", leftLift.getCurrentPosition(), rightLift.getCurrentPosition());
            telemetry.addData("","B Pressed: ", gamepad1.b);
            telemetry.addData("", "");
            telemetry.update();
        }
    }

    // Function lowers both
    public void lowerAll(int numTicks) {
        while(leftLift.getCurrentPosition() != numTicks) {
            // Set Target Position to 1 revolution
            leftLift.setTargetPosition(numTicks);
            rightLift.setTargetPosition(numTicks);

            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set Power
            leftLift.setPower(1);
            rightLift.setPower(-1);
        }
        // Let while statement finish
        idle();
        // Set both motor power to 0
        leftLift.setPower(0);
        rightLift.setPower(0);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset Encoders for every time button is pressed
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
