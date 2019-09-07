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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opsssssssssssssmode to the Driver Station OpMode list
 */

@Autonomous (name="CraterAlliance", group="Pushbot")
//@Disabled
public class TestCorrectionSampling extends LinearOpMode {

    /* Declare OpMode members. */
    //HardwareRobot         robot   = new HardwareRobot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.7;
    static final double ARM_SPEED = 0.9;
    static final double ACTUATOR_SPEED = 1;
    static final int LEFT=1;
    static final int CENTER=2;
    static final int RIGHT=3;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private boolean found = false;
    private int direction = RIGHT;

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
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AV/aIFL/////AAABmUDUvug7/UpynlsDkQH5uAQnsJuOz/6JBq/5SBey3OGv8TSO63o3l9ivyX1I4aWX6p/w3CfjkTWPJY8FxncF7GL+TTK3hdnOzNGwtMHfV6S6kL5mzQfStGMNK8FCG44I8P8HdpIcoxCvzaTcbk39NYvOV/IE12tJTqoOHDbX7/IpelOlSpXxO+GVI3n0eoxCdppbxFh+/XnOoCFbVKtSAFBOmVforWMHwryBloDB7XtBT18BAH5GXWzN4sRmVhDTDzZ0kBkZarHUNvyOwgeDZKCBGJwK6rHAYH8NFgU9FnoZ7oAA6R+jklhvHC+4rmWp84TFF7WM0uKVSuOw5LFZmMXALpmH1b0AMKSs/LhHo/Fd";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    /*public void landing(){
        actuatorTime(ACTUATOR_SPEED, .75);
        encoderDrive(DRIVE_SPEED, 5,5,7);
        encoderDrive(TURN_SPEED, 1,-1,5);
        encoderDrive(DRIVE_SPEED, -1,-1,5);
        encoderDrive(TURN_SPEED,3,-3,5);
    }

    public void center() {
        landing();
        encoderDrive(DRIVE_SPEED, -21, -21, 5.0);
        encoderDrive(TURN_SPEED, 6, -6, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 10, 10, 5.0);
        encoderDrive(TURN_SPEED, 4, -4, 5.0);
        encoderDrive(DRIVE_SPEED, -40, -40, 10.0);
        encoderDrive(TURN_SPEED, 5.5, -5.5, 5.0);
        encoderDrive(DRIVE_SPEED,-50, -50, 10.0); //used to be .2 less
        encoderArm2(ARM_SPEED, 0.7, 1);
        timeDrive(1, 1, 1.9);//^^
        timeDrive(1, -1, 1.3);
        timeDrive(-1, -1, 1.5);

    }
    public void left() {
        landing();
        encoderDrive(TURN_SPEED,3.5,-3.5,20);
        encoderDrive(DRIVE_SPEED, -21, -21, 5);
        encoderDrive(.3, 9.5, -9.5, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -30, -30, 10.0);
        encoderDrive(.3, 5, -5, 5.0);
        encoderDrive(DRIVE_SPEED,-45, -45, 10.0); //used to be .2 less
        encoderArm2(ARM_SPEED, 0.7, 1);
        timeDrive(1, 1, 2.1);//^^
        timeDrive(1, -1, 1.2);
        timeDrive(-1, -1, 1.5);

    }
    public void right() {
        landing();
        encoderDrive(TURN_SPEED,-3,3,20);
        encoderDrive(DRIVE_SPEED, -23.5, -23.5, 5);
        encoderDrive(.3, -24, 24, 10.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -50, -50, 10.0);
        encoderDrive(TURN_SPEED, 3.5, -3.5, 5.0);
        encoderDrive(DRIVE_SPEED,-47, -47, 10.0); //used to be .2 less
        encoderArm2(ARM_SPEED, 0.7, 1);
        timeDrive(1, 1, 1.9);//^^
        timeDrive(1, -1, 1.4);
        timeDrive(-1, -1, 1.4);

    }
    */


    @Override
    public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /* Wait for the game to begin */
        telemetry.addData(">", "Sampling");
        telemetry.update();


        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }

        ElapsedTime trackTime = new ElapsedTime();
        trackTime.reset();
        telemetry.addData("found=", found );
        while (found == false && trackTime.time() <= 3){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        //float goldMineralWidth = -1;
                        //float goldMineralHeight = -1;
                        //float goldMineralSize = -1;
                        float goldMineralLeft = -1;
                        float goldMineralTop = -1;
                        float goldMineralBottom = -1;
                        float goldMineralRight = -1;
                        float goldMineralWidth = -1;
                        float goldMineralHeight = -1;
                        //int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                                //goldMineralWidth = recognition.getWidth();
                                //goldMineralHeight = recognition.getHeight();
                                //goldMineralSize = recognition.getLeft();
                                goldMineralLeft = recognition.getLeft();
                                goldMineralTop = recognition.getTop();
                                goldMineralBottom = recognition.getBottom();
                                goldMineralRight = recognition.getRight();
                                goldMineralHeight = recognition.getHeight();
                                goldMineralWidth = recognition.getWidth();


                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            }
                        }

                        if (goldMineralX!=-1) {
                            if (goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                telemetry.update();
                                direction = LEFT;
                                found = true;

                            } else if (goldMineralX > silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                telemetry.update();
                                direction = CENTER;
                                found = true;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Right");
                                telemetry.update();
                                found = true;
                                direction = RIGHT;
                            }
                            telemetry.addData("Gold Position", goldMineralX);
                            telemetry.addData("Silver1 Position", silverMineral1X);
                            Log.d("ThunderBot", "Gold position" + goldMineralX );
                            Log.d("ThunderBot", "Silver1 Position" + silverMineral1X);
                            Log.d("ThunderBot", "GM Left" + goldMineralLeft + "GM Top" + goldMineralTop + "GM Bottom" + goldMineralBottom + "GM Right" + goldMineralRight + "GM Width" + goldMineralWidth + "GM Height" + goldMineralHeight );

                        }

                        found = true;


                    }
                }
            }
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        //robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0",  "Starting at %7d :%7d",
        //robot.leftDrive.getCurrentPosition(),
        //robot.rightDrive.getCurrentPosition());
        //telemetry.update();

        /*robot.armMotor.setPower(0);
        if (direction == LEFT) {
            left();
        } else if ( direction == CENTER) {
            center();
        } else if (direction == RIGHT) {
            right();
        }



     /*   robot.scoopServo.setPosition(1.0);            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move
        */


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderArm2(double speed, double power, double armtime) {
        ElapsedTime time_arm = new ElapsedTime();
        time_arm.reset();
        //robot.armMotor.setPower(power);
        //while (opModeIsActive() && (time_arm.seconds() < armtime) {
        //robot.armMotor.isBusy()) {
        //}

        //robot.armMotor.setPower(0);
    }
    public void timeDrive(double leftspeed, double rightspeed, double drivetime) {
        ElapsedTime time_drive = new ElapsedTime();
        time_drive.reset();

        //robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.leftDrive.setPower(leftspeed);
        //robot.rightDrive.setPower(rightspeed);

        while (opModeIsActive() && time_drive.seconds() < drivetime)  {

        }

        //robot.leftDrive.setPower(0);
        // robot.rightDrive.setPower(0);
    }
    public void actuatorTime(double speed, double time) {
        ElapsedTime time_actuator = new ElapsedTime();
        time_actuator.reset();

        //robot.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.leftDrive.setPower(speed);

        while (opModeIsActive() && time_actuator.seconds() < time)  {

        }

        //robot.hangMotor.setPower(0);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            /*newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            // robot.leftDrive.setPower(Math.abs(speed));
            //robot.rightDrive.setPower(Math.abs(speed));

            robot.leftDrive.setPower(DRIVE_SPEED);
            robot.rightDrive.setPower(DRIVE_SPEED);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

*/        }
    }
}

