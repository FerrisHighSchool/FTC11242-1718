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

package org.firstinspires.ftc.robotcontroller.external.samples;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


@TeleOp(name = "Sensor: Color", group = "Sensor")
@Disabled
public class SensorColor extends LinearOpMode {

  NormalizedColorSensor colorSensor;

  View relativeLayout;

  @Override public void runOpMode() throws InterruptedException {


    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    try {
      runSample();
    } finally {

      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.WHITE);
        }
      });
      }
  }

  protected void runSample() throws InterruptedException {


    float[] hsvValues = new float[3];
    final float values[] = hsvValues;


    boolean bPrevState = false;
    boolean bCurrState = false;


    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");


    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight)colorSensor).enableLight(true);
    }


    waitForStart();


    while (opModeIsActive()) {

      bCurrState = gamepad1.x;


      if (bCurrState != bPrevState) {

        if (bCurrState) {
          if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight)colorSensor;
            light.enableLight(!light.isLightOn());
          }
        }
      }
      bPrevState = bCurrState;


      NormalizedRGBA colors = colorSensor.getNormalizedColors();



      Color.colorToHSV(colors.toColor(), hsvValues);
      telemetry.addLine()
              .addData("H", "%.3f", hsvValues[0])
              .addData("S", "%.3f", hsvValues[1])
              .addData("V", "%.3f", hsvValues[2]);
      telemetry.addLine()
              .addData("a", "%.3f", colors.alpha)
              .addData("r", "%.3f", colors.red)
              .addData("g", "%.3f", colors.green)
              .addData("b", "%.3f", colors.blue);

      /** We also display a conversion of the colors to an equivalent Android color integer.
       * @see Color */
      int color = colors.toColor();
      telemetry.addLine("raw Android color: ")
              .addData("a", "%02x", Color.alpha(color))
              .addData("r", "%02x", Color.red(color))
              .addData("g", "%02x", Color.green(color))
              .addData("b", "%02x", Color.blue(color));


      float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
      colors.red   /= max;
      colors.green /= max;
      colors.blue  /= max;
      color = colors.toColor();

      telemetry.addLine("normalized color:  ")
              .addData("a", "%02x", Color.alpha(color))
              .addData("r", "%02x", Color.red(color))
              .addData("g", "%02x", Color.green(color))
              .addData("b", "%02x", Color.blue(color));
      telemetry.update();

      // convert the RGB values to HSV values.
      Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);


      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });
    }
  }
}
