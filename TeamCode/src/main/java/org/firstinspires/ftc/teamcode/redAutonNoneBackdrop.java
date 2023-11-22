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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hooSensing.SkystoneDeterminationPipeline.SkystonePosition;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.hardware.Servo;

//hi guys ryan hoo here please read all the comments !!!!!

//-----------Program Information

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


//-----------Other information
//FIELD IS 144x144 INCHES - READ THIS PLEASE - RYAN HOO
//frank pls read all comments - ryan hooman
//FIELD IS 144x144 INCHES - READ THIS PLEASE - RYAN HOO
//frank pls read all comments - ryan hooman
//FIELD IS 144x144 INCHES - READ THIS PLEASE - RYAN HOO
//frank pls read all comments - ryan hooman
//FIELD IS 144x144 INCHES - READ THIS PLEASE - RYAN HOO
//frank pls read all comments - ryan hooman
//FIELD IS 144x144 INCHES - READ THIS PLEASE - RYAN HOO
//frank pls read all comments - ryan hooman
//FIELD IS 144x144 INCHES - READ THIS PLEASE - RYAN HOO
//frank pls read all comments - ryan hooman
//if you do not read the comments i will come for you - RYLAN CHINTADA
// MEASUREMENTS ARE IN INCHES


//-----------Stuff to do:
//Check the static final double variables based on the robot parts.
//Check setPosition() arguments/parameters at end of autonomous.
//Check drive speed and turn speed.
//Check all functions.
//Code the autonoumous for the field, don't guess the position-- calculate it (144x144 inch field)
//Fix any other errors. NOTE: WHEN INSPECTING PROJECT ERRORS, IGNORE THE 2 ANDROID ERRORS


@Autonomous(name="redAutonNoneBackdrop", group="Robot")
@Disabled
public class redAutonNoneBackdrop extends LinearOpMode {


    /* Initialize motors as variables.... */
    private DcMotor         leftFront   = null;
    private DcMotor         rightFront  = null;
    private DcMotor         leftRear    = null;
    private DcMotor         rightRear   = null;
    private DcMotor         leftArm     = null;
    private DcMotor         rightArm    = null;
    private Servo           gripper     = null;
    //private DcMotor         intakeMotor = null;

    /* Other variables to initialize... */
    private ElapsedTime     runtime = new ElapsedTime();
    OpenCvInternalCamera theWebcam;
    hooSensing.SkystoneDeterminationPipeline pipeline;


    /*Variables based on robot parts, adjust whenever a part is changed */
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: Yellow Jacket 435 RPM Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing. //13.71 with gearing on 435 RPM YJ Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 3.779 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.5;
    static final double     SKIRT_SPEED             = 0.2;

    //FIELD IS 144x144 INCHES - READ THIS PLEASE - RYAN HOO

    @Override
    public void runOpMode() {

        /*Initialize the drive system variables.*/
        // Wheels
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        //Arm and Gripper
        leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm = hardwareMap.get(DcMotor.class, "rightArm");
        gripper = hardwareMap.get(Servo.class, "gripper");

        //Intake Motor
        //intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        //Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        theWebcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new hooSensing.SkystoneDeterminationPipeline();
        theWebcam.setPipeline(pipeline);
        theWebcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        theWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                theWebcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT); //upright, change for dif camera orientation

            }

            //Error message for driver hub if camera fails.
            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed");
                telemetry.update();

            }
        });

        /*Setting Directions for movement based on encoders*/
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Uncomment when testing arms
        //rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition());

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();





        //START OF AUTONOMOUS-----------------------------------------------------------------------------
        //ROBOT IS FACING INWARD TOWARDS PROP LINE

        /*
        //Close Gripper
        closeGripper();

        //Drive Foward 12 inches
        encoderDrive(DRIVE_SPEED, 12, 12, 5.0 );

        //Sense Prop Position
        hooSensing.SkystoneDeterminationPipeline.SkystonePosition skystonePosition = getSkystonePosition();
        telemetry.addData("Skystone Position", skystonePosition);
        telemetry.update();

        //Drive Backward 6 inches to rotate robot
        encoderDrive(DRIVE_SPEED, -6, -6, 5.0 );

        //Turn right twice to rotate robot
        encoderDrive(TURN_SPEED, 12, -12, 4.0);
        encoderDrive(TURN_SPEED, 12, -12, 4.0);


        //Place Pixel on same Line as Prop
        switch (skystonePosition) {
            case LEFT:
                //Skirt Left
                skirtLeft(3, 0.3);

                //Raise Both Arms
                raiseLeftArm(1.0, 2.5);
                raiseRightArm(1.0, 2.5);

                //Open Gripper
                openGripper();

                //Close Gripper
                closeGripper();

                //Lower Both Arms
                lowerLeftArm(1.0, 2.5);
                lowerRightArm(1.0, 2.5);

                //Skirt back to original position
                skirtRight(3, 0.3);

                break;
            case CENTER:
                //Move Foward 5 inches
                encoderDrive(DRIVE_SPEED, 5, 5, 5.0);

                //Raise Both Arms
                raiseLeftArm(1.0, 2.5);
                raiseRightArm(1.0, 2.5);

                //Open Gripper
                openGripper();

                //Close Gripper
                closeGripper();

                //Lower Both Arms
                lowerLeftArm(1.0, 2.5);
                lowerRightArm(1.0, 2.5);

                //Move Back to original position
                encoderDrive(DRIVE_SPEED, -5, -5, 5.0);

                break;
            case RIGHT:
                //Skirt Right
                skirtRight(3, 0.3);

                //Raise Both Arms
                raiseLeftArm(1.0, 2.5);
                raiseRightArm(1.0, 2.5);

                //Open Gripper
                openGripper();

                //Close Gripper
                closeGripper();

                //Lower Both Arms
                lowerLeftArm(1.0, 2.5);
                lowerRightArm(1.0, 2.5);

                //Skirt back to original position
                skirtLeft(3, 0.3);

                break;
        }

        //Turn right twice to rotate robot
        encoderDrive(TURN_SPEED, 12, -12, 4.0);
        encoderDrive(TURN_SPEED, 12, -12, 4.0);

        //Drive Backward 8 inches (To not collide with lines)
        encoderDrive(DRIVE_SPEED, -8, -8, 5.0);

        //Turn Right
        encoderDrive(TURN_SPEED, 12, -12, 4.0);

        //Drive Foward 12 Inches
        encoderDrive(DRIVE_SPEED, 12, 12, 3.0);

        // Turn Left
        encoderDrive(TURN_SPEED, -12, 12, 4.0);

        //Drive Foward 6 Inches
        encoderDrive(DRIVE_SPEED, 6, 6, 5.0);

        //Turn Right
        encoderDrive(TURN_SPEED, 12, -12, 4.0);


        */
        //Drive Foward 12 Inches
        encoderDrive(DRIVE_SPEED, 12, 12, 5.0);

        //REACHED BACKDROP--



    }


    //END OF AUTONOMOUS-----------------------------------------------------------------------------





    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     * -timeoutS is the maximum amount of time for the robot to reach the position,
     *  if it does not reach it in time method will stop and go onto next step in autonomous - ryan hooman !!!!
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightRearTarget = rightRear.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftRear.setTargetPosition(newLeftRearTarget);
            rightRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftRear.setPower(Math.abs(speed));
            rightRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(1000);   // optional pause after each move.
        }
    }

    //function to get position of prop, uses .getAnalysis() in hooSensing
    private hooSensing.SkystoneDeterminationPipeline.SkystonePosition getSkystonePosition() {
        // Call the pipeline's getAnalysis() method to obtain the latest Skystone position
        return pipeline.getAnalysis();
    }

    //Hmm i wonder wat this does
    private void raiseLeftArm(double power, double time) { //, int targetPosition (add for encoders), remove double time
        leftArm.setPower(power);  // Set the power to a negative value for downward motion
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            // Wait for the specified duration
        }
        leftArm.setPower(0);      // Stop the motor

        //with encoders:
        /*
        leftArm.setTargetPosition(targetPosition);
        lefttArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(power);
        while (opModeisActive() && rightArm.isBusy()) {
            // Wait for the arm to reach the target position
        }
        leftArm.setPower(0);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        sleep(1000);
    }

    //Hmm i wonder wat this does
    private void lowerLeftArm(double power, double time) { //, int targetPosition (add for encoders), remove double time
        leftArm.setPower(-power);  // Set the power to a negative value for downward motion
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            // Wait for the specified duration
        }
        leftArm.setPower(0);      // Stop the motor
        //with encoders:
        /*
        leftArm.setTargetPosition(targetPosition);
        lefttArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(-power);
        while (opModeisActive() && rightArm.isBusy()) {
            // Wait for the arm to reach the target position
        }
        leftArm.setPower(0);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        sleep(1000);
    }

    //Hmm i wonder wat this does
    private void raiseRightArm(double power, double time) { //, int targetPosition (add for encoders), remove double time
        rightArm.setPower(power);  // Set the power to a negative value for downward motion
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            // Wait for the specified duration
        }
        rightArm.setPower(0);      // Stop the motor
        //with encoders:
        /*
        rightArm.setTargetPosition(targetPosition);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(power);
        while (opModeisActive() && rightArm.isBusy()) {
            // Wait for the arm to reach the target position
        }
        rightArm.setPower(0);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        sleep(1000);
    }

    //Hmm i wonder wat this does
    private void lowerRightArm(double power, double time) { //, int targetPosition (add for encoders), remove double time
        rightArm.setPower(-power);  // Set the power to a negative value for downward motion
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            // Wait for the specified duration
        }
        rightArm.setPower(0);      // Stop the motor

        //with encoders:
        /*
        rightArm.setTargetPosition(targetPosition);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setPower(-power);
        while (opModeisActive() && rightArm.isBusy()) {
            // Wait for the arm to reach the target position
        }
        rightArm.setPower(0);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        sleep(1000);
    }

    //move robot right without turning with encoder
    public void skirtRight(double distance, double power) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Determine new target positions
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newRightFrontTarget = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newLeftRearTarget = leftRear.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newRightRearTarget = rightRear.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        // Set target positions
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(power);

        // Wait until motors reach the target position
        while (opModeIsActive() &&
                (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())) {
            telemetry.addData("Path", "Running to %7d : %7d", newLeftFrontTarget, newRightFrontTarget);
            telemetry.update();
        }

        // Stop the motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);
    }

    //move robot left without turning with encoder
    public void skirtLeft(double distance, double power) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Determine new target positions
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newRightFrontTarget = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newLeftRearTarget = leftRear.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newRightRearTarget = rightRear.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        // Set target positions
        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftRear.setTargetPosition(newLeftRearTarget);
        rightRear.setTargetPosition(newRightRearTarget);

        // Turn On RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power with reversed direction
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(-power);

        // Wait until motors reach the target position
        while (opModeIsActive() &&
                (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())) {
            telemetry.addData("Path", "Running to %7d : %7d", newLeftFrontTarget, newRightFrontTarget);
            telemetry.update();
        }

        // Stop the motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);
    }

    //Hmm i wonder wat this does
    private void openGripper(){
        gripper.setPosition(0);
        sleep(1000);
    }

    private void closeGripper() {
        gripper.setPosition(1);
        sleep(1000);
    } //0.13

    /*
    //Method to start intake motor
    private void startIntake() {
        intakeMotor.setPower(1.0);
    }

    //Method to stop intake motor
    private void stopIntake() {
        intakeMotor.setPower(0.0);
    }
    */


}
