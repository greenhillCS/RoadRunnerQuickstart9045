package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="centerstage", name="forShowAuton")
public class forShowAuton extends LinearOpMode {
    public void runOpMode(){
        double[] startingPos = {-36.0, 70.0-(DriveConstants.BOT_LENGTH/2)};
        double startHeading = 90;
        Pose2d start = new Pose2d(startingPos[0], startingPos[1], Math.toRadians(startHeading));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(start);

        TrajectorySequence t = drive.trajectorySequenceBuilder(start)
                .lineTo(new Vector2d(startingPos[0], startingPos[1] - 25))
                .turn(Math.toRadians(360))
                .lineToSplineHeading(new Pose2d(20, startingPos[1] - 25, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(48, startingPos[1] - 25, 0))
                .lineToSplineHeading(new Pose2d(30, startingPos[1] - 25, Math.toRadians(179)))
                .waitSeconds(1)
                .lineTo(new Vector2d(-61, startingPos[1] - 25))
                .lineTo(new Vector2d(10, startingPos[1] - 25))
                .lineTo(new Vector2d(48, startingPos[1] - 25))
                .lineTo(new Vector2d(48, startingPos[1] - 4))
                .lineTo(new Vector2d(60, startingPos[1] - 4))
                .build();

        waitForStart();

        drive.followTrajectorySequence(t);
    }
}
