/* Copyright (c) 2023 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//Vision and Roadrunner
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.VisionEasyOpenCV;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.openftc.easyopencv.OpenCvCameraFactory;

import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="AutoTestDrive")

public class AutoEasy extends LinearOpMode
{
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public enum ROLLING_DICE{
        LEFT_POS,
        CENTER_POS,
        RIGHT_POS
    }

    //How to determine where we start?
    public static START_POSITION startPosition;
    //What is the assigned position?
    public static ROLLING_DICE assignedPose;
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)


    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag





    @Override public void runOpMode()
    {

        // Initialize the Apriltag Detection process
        initAprilTag();

        //Define vision OpenCV
        VisionEasyOpenCV visionEasyOpenCV;
        OpenCvCamera cameraleft;
        OpenCvCamera cameraright;


        //Define the webcams
        String leftcam = "Webcam1";
        String rightcam = "Webcam2";
        visionEasyOpenCV = new VisionEasyOpenCV();

        DriveTrain robot;
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LF");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LB");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RB");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Selected Starting Position
        selectStartingPosition();

        //Initialize the camera on INIT
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cameraleft = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, leftcam), cameraMonitorViewId);
        cameraleft.setPipeline(visionEasyOpenCV);
        cameraleft.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                cameraright = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, leftcam), cameraMonitorViewId);
        cameraright.setPipeline(visionEasyOpenCV);
        cameraright.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cameraleft.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                cameraright.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        // Let driver prepare for Autonomous Starting
        // You got this!!!
        while (!isStopRequested() && !opModeIsActive()) {
            //Run visionEasyOpenCV.getPosition() and keep watching for the identifier in the Signal Cone.
            telemetry.clearAll();
            telemetry.addData("Starting RoboRaiders Autonomous Easy, type 2 for Left, 1 for Middle, and 0 for Right", "21386");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Park Position Identified by Camera: ", visionEasyOpenCV.getPosition());
            telemetry.update();
        }

        VisionEasyOpenCV.ParkingPosition position = VisionEasyOpenCV.ParkingPosition.ONE;

        while (opModeIsActive())
        {
            position = visionEasyOpenCV.getPosition();
            cameraleft.stopStreaming();
            cameraright.stopStreaming();

            if (assignedPose == ROLLING_DICE.LEFT_POS){
                moveLeft();
            }
            else if (assignedPose == ROLLING_DICE.RIGHT_POS){
                moveRight();
            }
            else{
                moveCenter();
            }
            //pseudocode for aditi
            /*
            command robot to move forward and turn left with encoder
            use webcam detection to determine where to place pixel
            if (assignedPose == "Left")
            {
            place pixel on left line
            move backwards
            turn towards the backdrop
            scan April tags
            place yellow pixel on left side of backdrop
            +45 points
            }
             */
        }
    }
    //Pose 2ds
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d leftPose;
    Pose2d rightPose;
    Pose2d centerPose;

    private void moveLeft() {
        /*
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        sleep(100);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
         */
        initPose = new Pose2d(-70, 32, Math.toRadians(0));
        midWayPose = new Pose2d(-42, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
        leftPose = new Pose2d(-40, 60, Math.toRadians(0)); // Left Location
    }

    private void moveRight() {
        initPose = new Pose2d(-70, 32, Math.toRadians(0));
        midWayPose = new Pose2d(-42, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
        rightPose = new Pose2d(-40, 11, Math.toRadians(0)); // Location 3
    }

    private void moveCenter() {
        initPose = new Pose2d(-70, 32, Math.toRadians(0));
        midWayPose = new Pose2d(-42, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
        centerPose = new Pose2d(-38, 35, Math.toRadians(0)); // Location 2
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
   /* public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }



        Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;


    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }


        Make sure camera is streaming before we try to set the exposure controls
       if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

        */

    TrajectorySequence trajectoryParking ;
    TrajectorySequence trajectoryParkingONE ;
    TrajectorySequence trajectoryParkingTHREE ;
    TrajectorySequence trajectoryParkingTWO ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    //Pose2d midWayPose;
    //Pose2d parkPose;
    //Pose2d firstPose;

    public void buildParking(VisionEasyOpenCV.ParkingPosition position){
        initPose = new Pose2d(-70, 32, Math.toRadians(0));
    }

    private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();
    }
    public void DriveForward(){

    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing RoboRaiders AutoOnlyPark for Team:","21386");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue LEFT   ", "(X)");
            telemetry.addData("    Blue RIGHT ", "(Y)");
            telemetry.addData("    Red LEFT    ", "(B)");
            telemetry.addData("    Red RIGHT  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();

            telemetry.addData("Processing...", "");
            telemetry.addData("...........................................", "");
            telemetry.addData("Select Randomized Path: ", "");
            telemetry.addData("Left ", "(X)");
            telemetry.addData("Center ", "(B)");
            telemetry.addData("Right ", "(Y)");

            if(gamepad1.x){
                assignedPose = ROLLING_DICE.LEFT_POS;
                break;
            }
            if(gamepad1.y){
                assignedPose = ROLLING_DICE.RIGHT_POS;
                break;
            }
            if(gamepad1.b){
                assignedPose = ROLLING_DICE.CENTER_POS;
                break;
            }
        }
        telemetry.clearAll();
    }

    }
