package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="intothedeep", name="allianceSpikeMarkAuton")
public class allianceSpikeMarkAuton extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(-90.00)));

        waitForStart();

        TrajectorySequence t = drive.trajectorySequenceBuilder(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(5.4, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(36, -13 , Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(46,-10, Math.toRadians(-90)))
                .lineTo(new Vector2d(47, -57))
                .lineTo(new Vector2d(47, -10))
                .lineToSplineHeading(new Pose2d(56, -10 , Math.toRadians(-90)))
                .lineTo(new Vector2d(56, -57))
                .lineToSplineHeading(new Pose2d(58, -10 , Math.toRadians(-90)))
                .lineTo(new Vector2d(63, -10))
                .lineTo(new Vector2d(63, -57))
                .build();

        drive.followTrajectorySequence(t);

        PositionStorage.pose = drive.getPoseEstimate();
    }
}
