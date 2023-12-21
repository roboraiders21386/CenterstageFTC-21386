/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OpModes.autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import static org.firstinspires.ftc.teamcode.drive.MecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VisionOpenCVPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/**
 * RR Autonomous
 */
@Autonomous(name = "RR Auto (roadrunner 0.5.6)", group = "00-Autonomous", preselectTeleOp = "RR TeleOp")
public class RRAutonomous extends LinearOpMode {

    public static String TEAM_NAME = "RoboRaiders";
    public static int TEAM_NUMBER = 21386;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private VisionPortal visionPortal;

    //Vision pipeline related - 11/24
    private VisionOpenCVPipeline visionPipeline;
    private OpenCvCamera camera;
    private String webcamName = "Webcam 1";


    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public enum ALLIANCE{
        BLUE,
        RED
    }
    public static START_POSITION startPosition;
    public static ALLIANCE alliance;


    public DriveTrain driveTrain;

    public Servo INTAKE;
    public Servo wrist;

    public String whichSide = "RIGHT";
    double ServoPosition = 1;
    double ServoSpeed = 0.5;
    private double TURN_WRIST = 0.5; //turn it forward
    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d moveBeyondTrussPose;
    Pose2d dropPurplePixelPose;
    Pose2d midwayPose1, midwayPose1a, intakeStack, midwayPose2, dropYellowPixelPose,parkPose;
    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryDrop, trajectoryParking, trajectoryFinal;

    @Override
    public void runOpMode() throws InterruptedException {

        INTAKE = hardwareMap.get(Servo.class, "INTAKE");
        wrist = hardwareMap.get(Servo.class, "WRIST");
        //Create the vision pipeline object - 11/24
        visionPipeline = new VisionOpenCVPipeline(telemetry);
        driveTrain = new DriveTrain(hardwareMap);


        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        //Activate Camera Vision
        visionPipeline.setAlliancePipe(String.valueOf(alliance));
        initVision(visionPipeline);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Vision identified Parking Location", visionPipeline.getSide());
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            //Build parking trajectory based on last detected target by vision
            whichSide = visionPipeline.getSide();
            camera.stopStreaming();


            //set the starting pose
            initPose = new Pose2d(0, 0, 0); // Starting Pose
            moveBeyondTrussPose = new Pose2d(25,0,0);
            dropPurplePixelPose = new Pose2d(37, 0, 0);
            //midwayPose1 = new Pose2d(0,0,0);
            midwayPose1a = new Pose2d(0,0,0);
            intakeStack = new Pose2d(0,0,0);
            midwayPose2 = new Pose2d(0,0,0);
            dropYellowPixelPose = new Pose2d(30, 55, Math.toRadians(90));
            parkPose = new Pose2d(0,0, 0);

            //Letting the drivetrain know...
            driveTrain.getLocalizer().setPoseEstimate(initPose);

            switch (startPosition) {
                case BLUE_LEFT:
                    setBlueLeftPositions(whichSide);
                    break;
                case RED_RIGHT:
                    setRedRightPositions(whichSide);
                    break;
                case RED_LEFT:
                    //TO DO
                    setRedLeftPositions(whichSide);
                    break;
                case BLUE_RIGHT:
                    //TO DO
                    setBlueRightPositions(whichSide);
                    break;
            }
            //setBlueLeftPositions(whichSide);

            runTestAutonomous();
            driveTrain.followTrajectorySequence(trajectoryFinal);
            INTAKE.setDirection(Servo.Direction.FORWARD);
            INTAKE.setPosition(0.75);
            sleep(1000);
            INTAKE.setPosition(0);
        }
    }   // end runOpMode()


    public void runTestAutonomous(){
        trajectoryDrop = driveTrain.trajectorySequenceBuilder(initPose)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(moveBeyondTrussPose)
                .lineToLinearHeading(dropPurplePixelPose)
                //.lineToLinearHeading(midwayPose1)
                //.lineToLinearHeading(dropYellowPixelPose)
                //.lineToLinearHeading(parkPose)
                //.back(10)
                .build();
        trajectoryParking = driveTrain.trajectorySequenceBuilder(trajectoryDrop.end())
                .setVelConstraint(getVelocityConstraint(60 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .back(15)
                .lineToSplineHeading(dropYellowPixelPose)
                .build();
        trajectoryFinal = driveTrain.trajectorySequenceBuilder(trajectoryParking.end())
                .forward(8)
                .build();
        driveTrain.followTrajectorySequence(trajectoryDrop);
        wrist.setDirection(Servo.Direction.FORWARD); //edit for only one signal bc of y cable
        wrist.setPosition(TURN_WRIST); //edit for only one signal bc of y cable
        ServoPosition += ServoSpeed;
        sleep(5000);
        driveTrain.followTrajectorySequence(trajectoryParking);

    }

    public void setBlueLeftPositions(String whichSide){
        moveBeyondTrussPose = new Pose2d(20,0,0);
        switch(whichSide){
            case "LEFT":
                //dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
                dropPurplePixelPose = new Pose2d(30, 8, Math.toRadians(45));
                //dropYellowPixelPose = new Pose2d(23, 36, Math.toRadians(-90));
                dropYellowPixelPose = new Pose2d(25, 38, Math.toRadians(90));
                break;
            case "CENTER":
                dropPurplePixelPose = new Pose2d(36, 0, Math.toRadians(0));
                dropYellowPixelPose = new Pose2d(34, 38,  Math.toRadians(90));
                break;
            case "RIGHT":
                dropPurplePixelPose = new Pose2d(32, -8, Math.toRadians(315));
                telemetry.addData("Right", whichSide);
                dropYellowPixelPose = new Pose2d(42, 38, Math.toRadians(90));
                break;
        }
        //midwayPose1 = new Pose2d(14, 13, Math.toRadians(-45));


    } //end of setBlueLeftPositions

    public void setRedRightPositions(String whichSide){
        //moveBeyondTrussPose = new Pose2d(30,0,0);


        //THESE ARE NOT THE CORRECT POSITIONS FOR RED RIGHT SO CHANGE THEM
        switch(whichSide){
            case "LEFT":
                //dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
                dropPurplePixelPose = new Pose2d(27, 18, Math.toRadians(0));
                telemetry.addData("Left", whichSide);
                //dropYellowPixelPose = new Pose2d(23, 36, Math.toRadians(-90));
                dropYellowPixelPose = new Pose2d(37, -36, Math.toRadians(270));
                break;
            case "CENTER":
                dropPurplePixelPose = new Pose2d(33, 0, Math.toRadians(0));
                dropYellowPixelPose = new Pose2d(30, -36,  Math.toRadians(270));
                break;
            case "RIGHT":
                dropPurplePixelPose = new Pose2d(27, -19, Math.toRadians(0));
                dropYellowPixelPose = new Pose2d(15, -36, Math.toRadians(270));
                break;
        }
        //midwayPose1 = new Pose2d(14, 13, Math.toRadians(-45));
        midwayPose1 = new Pose2d(12, 0, 0);
        //waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
        //parkPose = new Pose2d(8, 30, Math.toRadians(90));
        //double waitSecondsBeforeDrop = 0;

    }  // end of setRedRightPositions

    public void setRedLeftPositions(String whichSide){

    } //end of setRedLeftPositions

    public void setBlueRightPositions(String whichSide){

    } //end of setBlueRightPositions





    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech Gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                alliance = ALLIANCE.BLUE;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                alliance = ALLIANCE.BLUE;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                alliance = ALLIANCE.RED;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                alliance = ALLIANCE.RED;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }  // End selectStartingPosition

    public void setRedRightPos(){

    }
    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


    private void initVision(VisionOpenCVPipeline visionPipeline) {
        // Initiate Camera on INIT.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        camera.setPipeline(visionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);  this was used in PowerPlay
                //camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);  // this was used by ColorBlobAuto
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                while (!visionPipeline.hasProcessedFrame) sleep(50);

            }

            @Override
            public void onError(int errorCode) {}
        });
    }  // end of initVision()

}   // end class