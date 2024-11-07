package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="intothedeep", name="alliance Cycle AutonRight")
public class allianceCycleAutonRight extends LinearOpMode {
    TrajectorySequence entrance;
    TrajectorySequence cycle1;
    TrajectorySequence cycle2;
    TrajectorySequence cycle3;
    TrajectorySequence end;
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo clawServo = hardwareMap.get(Servo.class, "CS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor, telemetry);

        drive.setPoseEstimate(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)));

        clawServo.setPosition(1);
        slides.moveTo(10);

        entrance = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(slides::hookPosUp)
                .splineTo(new Vector2d(0, -26-(DriveConstants.BOT_LENGTH/2)), Math.toRadians(90))
                .build();

        cycle1 = drive.trajectorySequenceBuilder(entrance.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineTo(new Vector2d(41, -10), Math.toRadians(0))
                .splineTo(new Vector2d(46, -24), Math.toRadians(270))
                .forward(33)
                .splineToConstantHeading(new Vector2d(48, -72+(DriveConstants.BOT_LENGTH/2)), Math.toRadians(270))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(0, -26-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .build();

        cycle2 = drive.trajectorySequenceBuilder(cycle1.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineTo(new Vector2d(46, -10), Math.toRadians(0))
                .splineTo(new Vector2d(56, -24), Math.toRadians(270))
                .forward(33)
                .splineToConstantHeading(new Vector2d(48, -72+(DriveConstants.BOT_LENGTH/2)), Math.toRadians(270))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(0, -26-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .build();

        cycle3 = drive.trajectorySequenceBuilder(cycle2.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineTo(new Vector2d(48, -10), Math.toRadians(0))
                .splineTo(new Vector2d(60, -24), Math.toRadians(270))
                .forward(33)
                .splineToConstantHeading(new Vector2d(48, -72+(DriveConstants.BOT_LENGTH/2)), Math.toRadians(270))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(0, -26-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .build();

        end = drive.trajectorySequenceBuilder(cycle3.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(90))
                .splineTo(new Vector2d(41, -10), Math.toRadians(0))
                .splineTo(new Vector2d(46, -24), Math.toRadians(270))
                .forward(33)
                .splineToConstantHeading(new Vector2d(48, -72+(DriveConstants.BOT_LENGTH/2)), Math.toRadians(270))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(0, -26-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequenceAsync(entrance);
            drive.followTrajectorySequenceAsync(cycle1);
            drive.followTrajectorySequenceAsync(cycle2);
            drive.followTrajectorySequenceAsync(cycle3);
            drive.followTrajectorySequenceAsync(end);
            PositionStorage.pose = drive.getPoseEstimate();
            break;
        }
    }
}
