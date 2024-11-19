package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class yellowAutonTigerTest extends LinearOpMode {

    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo clawServo = hardwareMap.get(Servo.class, "CS");

        RevTouchSensor scoringTouchSensor = hardwareMap.get(RevTouchSensor.class, "TS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor, telemetry, scoringTouchSensor);

        drive.setPoseEstimate(new Pose2d(-19, -60+(DriveConstants.BOT_LENGTH/2), Math.toRadians(180)));

        clawServo.setPosition(1);
        slides.moveToWait(-10);
        while (slideMotor.isBusy() && slides.runTime.seconds() < slides.timeOutSecs && opModeIsActive() && !isStopRequested()) {
            continue;
        }

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d(-19, -60+(DriveConstants.BOT_LENGTH/2), Math.toRadians(180)))
                .splineTo(new Vector2d(60, -60-(DriveConstants.BOT_LENGTH/2)), Math.toRadians(180))
                .build();

        TrajectorySequence t2 = drive.trajectorySequenceBuilder(t1.end())
                .lineToSplineHeading(new Pose2d(-38.35, -32.22,  Math.toRadians(135)))
                .lineToSplineHeading(new Pose2d(-44, -28, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-60, -60, Math.toRadians(-135)))
                .lineToSplineHeading(new Pose2d(-44, -28, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-52, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-60, -65, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-54, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-60.5, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-61, -66, Math.toRadians(180)))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(t1);
            drive.followTrajectorySequence(t2);
            clawServo.setPosition(0);
            slides.moveToWait(0);
            PositionStorage.pose = drive.getPoseEstimate();
            break;
        }
    }
}
