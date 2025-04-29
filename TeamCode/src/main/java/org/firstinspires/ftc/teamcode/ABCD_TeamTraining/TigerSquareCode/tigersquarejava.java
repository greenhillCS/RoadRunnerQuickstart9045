package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.TigerSquareCode;

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

@Autonomous(name = "tigersquarecode")
public class tigersquarejava extends LinearOpMode {

    TrajectorySequence cycle1;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(60, -67+(DriveConstants.BOT_LENGTH/2), Math.toRadians(-90.00));


        cycle1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .lineTo(new Vector2d(60, 55))
                .lineTo(new Vector2d(-60, 55))
                .lineTo(new Vector2d(-60, -60))
                .lineTo(new Vector2d(60, -60))
                .build();

        drive.setPoseEstimate(startPos);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(cycle1);
        }
    }
}
