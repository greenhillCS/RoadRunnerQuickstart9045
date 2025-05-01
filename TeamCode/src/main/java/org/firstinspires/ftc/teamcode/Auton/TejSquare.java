package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import java.io.File;
import java.io.IOException;
import java.util.Vector;


@Autonomous(group="intothedeep", name="TejExceelectAmaazingSqaureAutonomweous")
public class TejSquare extends LinearOpMode {

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(53, -53, Math.toRadians(90.00));

        drive.setPoseEstimate(startPos);


        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(53,53))
                .turn(Math.toRadians(90))

                .lineTo(new Vector2d(-53,53))
                .turn(Math.toRadians(90))

                .lineTo(new Vector2d(-53,-53))
                .turn(Math.toRadians(90))

                .lineTo(new Vector2d(53,-53))
                .turn(Math.toRadians(90))
                .build();

        waitForStart();



        while (!isStopRequested() && opModeIsActive()) {
            drive.followTrajectorySequence(trajectory);
            break;
        }
    }
    }


