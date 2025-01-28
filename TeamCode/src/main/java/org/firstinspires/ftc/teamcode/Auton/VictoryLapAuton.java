package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="IntoTheDeep", name="VictoryLapAuton")
@Disabled
public class VictoryLapAuton extends LinearOpMode {
    private boolean firstLoop = true;
    private final Pose2d start = PositionStorage.pose;
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor hangingMotor = hardwareMap.get(DcMotor.class, "HM");
        DcMotor scoringMotor = hardwareMap.get(DcMotor.class, "SM");

        Servo claw = hardwareMap.get(Servo.class, "CS");

        TrajectorySequence startingTrajectory = drive.trajectorySequenceBuilder(start)
                        .lineTo(new Vector2d(0, -48))
                        .build();

        TrajectorySequence startingLoopingTrajectory = drive.trajectorySequenceBuilder(startingTrajectory.end())
                .lineToSplineHeading(new Pose2d(-48, -48,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-48, -24,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-48, 0,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-48, 24,Math.toRadians(270)))

                .lineToSplineHeading(new Pose2d(-48, 48,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-24, 48,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(0, 48,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(24, 48,Math.toRadians(270)))

                .lineToSplineHeading(new Pose2d(48, 48,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(48, 24,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(48, 0,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(48, -24,Math.toRadians(270)))

                .lineToSplineHeading(new Pose2d(48, -48,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(24, -48,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(0, -48,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-24, -48,Math.toRadians(270)))
                .build();

        TrajectorySequence loopingTrajectory = drive.trajectorySequenceBuilder(startingTrajectory.end())
                .lineToSplineHeading(new Pose2d(-48, -48,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-48, -24,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-48, 0,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-48, 24,Math.toRadians(270)))

                .lineToSplineHeading(new Pose2d(-48, 48,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-24, 48,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(0, 48,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(24, 48,Math.toRadians(270)))

                .lineToSplineHeading(new Pose2d(48, 48,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(48, 24,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(48, 0,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(48, -24,Math.toRadians(270)))

                .lineToSplineHeading(new Pose2d(48, -48,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(24, -48,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(0, -48,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-24, -48,Math.toRadians(270)))
                .build();

        waitForStart();
        while(opModeIsActive()){
            if (firstLoop){
                drive.followTrajectorySequence(startingTrajectory);
                drive.followTrajectorySequence(startingLoopingTrajectory);
                firstLoop = false;
            } else {
                drive.followTrajectorySequence(loopingTrajectory);
            }
        }
    }
}
