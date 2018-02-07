package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.InterruptedException;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.IOException;
import java.io.InterruptedIOException;

/**
 * Created by nerdxoverboard on 1/07/2018.
 */

public class CobaltStraight {
    /**
     * This OpMode illustrates the basics of using the Vuforia engine to determine
     * the identity of Vuforia VuMarks encountered on the field. The code is structured as
     * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
     * duplicate the core Vuforia documentation found there, but rather instead focus on the
     * differences between the use of Vuforia for navigation vs VuMark identification.
     *
     * @see ConceptVuforiaNavigation
     * @see VuforiaLocalizer
     * @see VuforiaTrackableDefaultListener
     * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
     *
     * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
     * is explained in {@link ConceptVuforiaNavigation}.
     */
    @Autonomous(name="Crimson: Turn", group ="Concept")
    //@Disabled
    public static class ConceptVuMarkIdentification extends LinearOpMode {
        DcMotor leftMotor,  // left drive wheel
                rightMotor, // right drive wheel
                leftTred,   // left tread
                rightTred;  // right tread
        public static final String TAG = "Vuforia VuMark Sample";
        OpenGLMatrix lastLocation = null;
        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        VuforiaLocalizer vuforia;
        @Override
        public void runOpMode() throws InterruptedException {
            leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
            rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
            leftTred = hardwareMap.get(DcMotor.class, "left_tred");
            rightTred = hardwareMap.get(DcMotor.class, "right_tred");
            /*
             * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
             * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            // OR...  Do Not Activate the Camera Monitor View, to save power
            // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            /*
             * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
             * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
             * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
             * web site at https://developer.vuforia.com/license-manager.
             *
             * Vuforia license keys are always 380 characters long, and look as if they contain mostly
             * random data. As an example, here is a example of a fragment of a valid key:
             *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
             * Once you've obtained a license key, copy the string from the Vuforia web site
             * and paste it in to your code onthe next line, between the double quotes.
             */
            parameters.vuforiaLicenseKey = "AV9/WxX/////AAAAGQbIdaIVK0m6iLz60XjMAQZpO4kPZE+Wjmvw8oV2JHyRTny4EO/n1OUvDeQOURF0mHd4aoEYqg3nig2ea4L1exx95Q12s6per2hn+My2M4MgKGe+thlUi8AVlFT5ePVmzwDdEk2ryoKNRuykplJnD1pIg1oq2B94jd6xIbvFOXx+FgIsLefO6FMaPfHt+uWiVao/en0kjsKH6LRqM1/C9Q0sZyUijmLHNvyfhgLKQgIlr34Tcuc9gUDe0DXN9y8P/3BOxWYP2dIUB4snzoy+Y8NXQLW5jOQeAwH+/beb4d74CADwUv08rP9QQG51pFnSFK3vS2Z4QPeJRIQnM87cj2XImhIumm5iBlnyMFHSHFRV";
            /*
             * We also indicate which camera on the RC that we wish to use.
             * Here we chose the back (HiRes) camera (for greater range), but
             * for a competition robot, the front camera might be more convenient.
             */
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            /**
             * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
             * in this data set: all three of the VuMarks in the game were created from this one template,
             * but differ in their instance id information.
             * @see VuMarkInstanceId
             */
            VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            waitForStart();

            relicTrackables.activate();

            while (opModeIsActive()) {

                /**
                 * See if any of the instances of {@link relicTemplate} are currently visible.
                 * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                 * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                 * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                 */
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        drive(1200);
                        turnAt(90, 1);
                        stop();
                    }
                    if (vuMark == RelicRecoveryVuMark.CENTER) {
                        drive(30, 0.5, 0.5);
                        turnAt(-90, 1);
                        stop();
                    }
                    if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        drive(1250);
                        turnAt(90, 1);
                        stop();
                    }

                    telemetry.addData("VuMark", "%s visible", vuMark);

                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }

                }
                else {
                    telemetry.addData("VuMark", "not visible");
                }
                telemetry.update();
            }

        }


        static final double COUNTS_PER_MOTOR_REV = 1120;
        static final double WHEEL_DIAMETER = 3.85; // for finding circumference, in inches
        static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER * Math.PI); //about 36.4 pulses per in


        // method for driving forward
        public void drive(double inches, double leftSpeed, double rightSpeed) throws InterruptedException {

            int newRightTarget;
            // Ensure that the opmode is still active

            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newRightTarget = rightMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

                leftMotor.setTargetPosition(newRightTarget);
                rightMotor.setTargetPosition(newRightTarget);

                // Turn On RUN_TO_POSITION
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // This is helpful if we want to reverse with negative inches
                if (inches > 0) {

                    leftMotor.setPower(leftSpeed);
                    rightMotor.setPower(rightSpeed);
                } else {

                    leftMotor.setPower(-leftSpeed);
                    rightMotor.setPower(-rightSpeed);
                }

                //loop for telemetry while the opmode is runnning
                while ((leftMotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newRightTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            leftMotor.getCurrentPosition(),
                            rightMotor.getCurrentPosition());

                    telemetry.update();
                    idle();
                }

                leftMotor.setPower(0);
                rightMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(250);
            }
        }



        public double turnAtAngle(double angle) {

            double inchesToTurn = (angle * (Math.PI / 180) * 18);

            return inchesToTurn;
        }


        // method for turning towards the right
        public void turnAt(double angle, double speed) throws InterruptedException {

            int leftMotorPosition = leftMotor.getCurrentPosition();
            int newAngleTarget = leftMotor.getCurrentPosition() + (int) (turnAtAngle(angle) * COUNTS_PER_INCH);

            if (opModeIsActive()) {

                leftMotor.setTargetPosition(newAngleTarget);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (angle > 0) {

                    leftMotor.setPower(speed);
                } else {
                    leftMotor.setPower(speed);
                }

                // telemetry for robot rotating at desired angle
                while (opModeIsActive() && leftMotor.isBusy()) {
                    // Display it for the driver.
                    telemetry.addData("Angle Turn", "Running to %7d", newAngleTarget);
                    telemetry.addData("Path2", "Running at " + " : " + leftMotorPosition);
                    telemetry.update();

                    // Allow time for other processes to run.
                    idle();
                }

                leftMotor.setPower(0);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(250);
            }
        }


        String format(OpenGLMatrix transformationMatrix) {
            return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        }


    }

}
