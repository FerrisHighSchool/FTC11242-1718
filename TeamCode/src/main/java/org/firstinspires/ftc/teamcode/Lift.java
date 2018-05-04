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
            boolean liftOne;
            boolean liftTwo;
            boolean lowerAll;
            int leftCurrent;
            int rightCurrent;

            double drive = -gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_y;
            liftTwo = gamepad1.a;
            lowerAll = gamepad1.b;
            treadsUp = gamepad1.dpad_up;
            treadsDown = gamepad1.dpad_down;

            if (liftTwo == true){
                //leftCurrent = leftLift.getCurrentPosition();
                //rightCurrent = rightLift.getCurrentPosition();
                //leftLift.setTargetPosition(leftCurrent + 500);
                //rightLift.setTargetPosition(rightCurrent + 500);
                leftLift.setTargetPosition(500);
                rightLift.setTargetPosition(500);
                leftLift.setPower(1);
                rightLift.setPower(1);
           /* }else if (liftTwo == false && leftLift.isBusy() == true){
                leftLift.setTargetPosition(500);
                rightLift.setTargetPosition(500);
                leftLift.setPower(1);
                rightLift.setPower(1);
                liftTwo = false;
            }else if (liftTwo == false && rightLift.isBusy() == true){
                leftLift.setTargetPosition(500);
                rightLift.setTargetPosition(500);
                leftLift.setPower(1);
                rightLift.setPower(1);
                liftTwo = false;
            */}else{
                leftLift.setPower(0);
                rightLift.setPower(0);
            }
            //boolean for tread rotations
            if(treadsDown == true){
                leftTred.setPower(-1);
                rightTred.setPower(1);
            }
            else if(treadsUp == true){
                leftTred.setPower(1);
                rightTred.setPower(-1);
            }else{
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
            telemetry.addData("Lift Trigger",liftTwo);
            telemetry.update();
        }
    }
}
