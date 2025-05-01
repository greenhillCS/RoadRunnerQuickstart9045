package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepIntakeSystem;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="intothedeep", name="Johnny Thing")
public class Johnny_Auton_Thing extends LinearOpMode {

    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(46, -46, 0);

        drive.setPoseEstimate(startPos);


        TrajectorySequence trajectory =  drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(46, 46))
                .lineTo(new Vector2d(-46, 46))
                .lineTo(new Vector2d(-46, -46))
                .lineTo(new Vector2d(46, -46))
                .build();

        waitForStart();



        while (!isStopRequested() && opModeIsActive()){
            drive.followTrajectorySequence(trajectory);
        }

    }
}
