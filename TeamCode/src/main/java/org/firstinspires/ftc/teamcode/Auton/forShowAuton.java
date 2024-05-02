package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.wideBot.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="centerstage", name="forShowAuton")
public class forShowAuton extends LinearOpMode {
    public void runOpMode(){
        double[] startingPos = {-36.0, 70.0-(DriveConstants.BOT_LENGTH/2)};
        double startHeading = -90;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence t = drive.trajectorySequenceBuilder(new Pose2d(startingPos[0], startingPos[1], startHeading))
                .lineTo(new Vector2d(startingPos[0], startingPos[1] - 27))
                .turn(Math.toRadians(360))
                .lineToSplineHeading(new Pose2d(15, startingPos[1] - 27, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(48, startingPos[1] - 27, 0))
                .lineToSplineHeading(new Pose2d(0, startingPos[1] - 27, Math.toRadians(180)))
                .lineTo(new Vector2d(-61, startingPos[1] - 27))
                .lineTo(new Vector2d(10, startingPos[1] - 27))
                .lineToSplineHeading(new Pose2d(48, startingPos[1] - 27, 0))
                .lineToSplineHeading(new Pose2d(48, startingPos[1] - 3, 0))
                .lineToSplineHeading(new Pose2d(60, startingPos[1] - 3, 0))
                .build();

        drive.followTrajectorySequence(t);
    }
}
