package org.firstinspires.ftc.teamcode.OpModes.autonomous;

import static org.firstinspires.ftc.teamcode.drive.MecanumDrive.getVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;


/**
 * RR Autonomous
 */
@Autonomous(name = "RR Test Auto for RR 0.56")
public class RRTestAuto extends LinearOpMode {

    public DriveTrain driveTrain;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new DriveTrain(hardwareMap);
        //Initialize any other TrajectorySequences as desired
        TrajectorySequence trajectoryParking ;

        //Initialize any other Pose2d's as desired
        Pose2d initPose; // Starting Pose
        Pose2d midWayPose;
        Pose2d parkPose;
        Pose2d firstPose;

        //MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Testing autonomous: ", "21386");
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //assuming we start on blue left
            initPose = new Pose2d(70, 32, Math.toRadians(180)); //Starting pose
            midWayPose = new Pose2d(-42, 36, Math.toRadians(0));
            parkPose = new Pose2d(38, 35, Math.toRadians(180));

            trajectoryParking = driveTrain.trajectorySequenceBuilder(initPose)
                    .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                    .forward(18)
                    //.lineToLinearHeading(midWayPose)
                    //.lineToLinearHeading(parkPose)
                    .build();
            driveTrain.getLocalizer().setPoseEstimate(initPose);
            driveTrain.followTrajectorySequence(trajectoryParking);
            telemetry.addData("Parked in Location", "Done");
            telemetry.update();
        } //opModeIsActive()
    }   // end runOpMode()



}   // end class
