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
 * Recreated by nerdxoverboard on 11/17/2017.
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

@TeleOp(name="Tred", group="Linear Opmode")

public class Tred extends LinearOpMode {

    //private OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor leftTred = null;
    private DcMotor rightTred = null;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;


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


            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            liftTwo = gamepad1.a;
            treadsUp = gamepad1.dpad_up;
            treadsDown = gamepad1.dpad_down;

            if (liftTwo == true) {
                /*leftLift.setPower(-0.5); // goes up, mirrors right lift
                rightLift.setPower(0.5); // negated to go the same direction as the other lift
                sleep(2000);*/

            }


            if(treadsUp == true){
                leftTred.setPower(1);
                rightTred.setPower(1);
            }
            else if(treadsDown == true){
                rightTred.setPower(-1);
                leftTred.setPower(-1);
            }else{
                rightTred.setPower(0);
                leftTred.setPower(0);
            }




            leftPower = Range.clip(drive + turn, -1.0, 1.0);//gage drive
            rightPower = Range.clip(drive - turn, -1.0, 1.0);//gage drive

            leftMotor.setPower(leftPower);  //motor power
            rightMotor.setPower(rightPower);  //motor power

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Treads", "left, right", treadsUp, treadsDown);
            telemetry.addData("Lift", "left, right", leftLift, rightLift);
            telemetry.update();
        }
    }
}
//}
