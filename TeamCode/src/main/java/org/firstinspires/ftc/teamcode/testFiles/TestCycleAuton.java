package org.firstinspires.ftc.teamcode.testFiles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(group="test", name="Test Cycle Auton")
@Disabled
public class TestCycleAuton extends LinearOpMode {
    enum Mode {
        MOVING,
        SCORE,
        INTAKE,
        NULL,
        STOP
    }
    Mode mode;
    TrajectorySequence auton;
    double slidePos;
    double targetPos;
    Pose2d lastPos;
    public void runOpMode(){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo clawServo = hardwareMap.get(Servo.class, "CS");

        RevTouchSensor scoringTouchSensor = hardwareMap.get(RevTouchSensor.class, "TS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor, telemetry, scoringTouchSensor);
        ElapsedTime time = new ElapsedTime();
        mode = Mode.MOVING;

        drive.setPoseEstimate(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)));

        clawServo.setPosition(1);
        slides.moveTo(10);
        while (slideMotor.isBusy() && !isStopRequested() && opModeIsActive()){
            continue;
        }

        auton = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-4, -24-(DriveConstants.BOT_LENGTH/2)), Math.toRadians(90))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -61), Math.toRadians(270))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(46, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -61), Math.toRadians(270))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(0, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(52, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(62, -50), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -61), Math.toRadians(270))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(48, -61, Math.toRadians(270)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(4, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(48, -61, Math.toRadians(270)))
                .build();

        drive.followTrajectorySequenceAsync(auton);

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && drive.isBusy()) {
            drive.update();
            slidePos = slideMotor.getCurrentPosition();
            targetPos = slideMotor.getTargetPosition();
        }
        PositionStorage.pose = drive.getPoseEstimate();
    }
}
