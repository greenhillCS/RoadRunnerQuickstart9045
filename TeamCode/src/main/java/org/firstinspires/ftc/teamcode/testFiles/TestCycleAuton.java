package org.firstinspires.ftc.teamcode.testFiles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(group="test", name="Test Cycle Auton")
public class TestCycleAuton extends LinearOpMode {
    enum Mode {
        MOVING,
        SCORE,
        INTAKE,
        NULL,
        STOP
    }
    Mode mode;
    TrajectorySequence entrance;
    TrajectorySequence scoring;
    TrajectorySequence cycle1;
    TrajectorySequence cycle2;
    TrajectorySequence cycle3;
    TrajectorySequence end;
    double slidePos;
    double targetPos;
    Pose2d lastPos;
    public void runOpMode(){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo clawServo = hardwareMap.get(Servo.class, "CS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor, telemetry);
        ElapsedTime time = new ElapsedTime();
        mode = Mode.MOVING;

        drive.setPoseEstimate(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)));

        clawServo.setPosition(1);
        slides.moveTo(10);
        while (slideMotor.isBusy() && !isStopRequested() && opModeIsActive()){
            continue;
        }

        entrance = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(slides::hookPosUp)
                .splineToConstantHeading(new Vector2d(-2, -24-(DriveConstants.BOT_LENGTH/2)), Math.toRadians(90))
                .addTemporalMarker(slides::hookPosDown)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{mode = Mode.SCORE;})
//                .addTemporalMarker(()->lastPos = entrance.end())
                .waitSeconds(0.5)
                .addTemporalMarker(()->drive.followTrajectorySequence(cycle1))
                .build();

//        scoring = drive.trajectorySequenceBuilder(lastPos)
//                .addTemporalMarker(slides::hookPosUp)
//                .addTemporalMarker(()->{mode = Mode.MOVING;})
//                .lineToLinearHeading(new Pose2d(-2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
//                .addTemporalMarker(slides::hookPosDown)
//                .addTemporalMarker(()->{mode = Mode.MOVING;})
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{mode = Mode.SCORE;})
////                .addTemporalMarker(()->lastPos = scoring.end())
//                .build();

        cycle1 = drive.trajectorySequenceBuilder(entrance.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -66), Math.toRadians(270))
                .addTemporalMarker(()->{mode = Mode.INTAKE;})
                .waitSeconds(0.5)
                .addTemporalMarker(slides::hookPosUp)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .lineToLinearHeading(new Pose2d(-2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .addTemporalMarker(slides::hookPosDown)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{mode = Mode.SCORE;})
                .waitSeconds(0.5)
//                .addTemporalMarker(()->lastPos = cycle1.end())
                .addTemporalMarker(()->drive.followTrajectorySequenceAsync(cycle2))
                .build();

        cycle2 = drive.trajectorySequenceBuilder(cycle1.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(46, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -66), Math.toRadians(270))
                .addTemporalMarker(()->{mode = Mode.INTAKE;})
                .waitSeconds(0.5)
                .addTemporalMarker(slides::hookPosUp)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .lineToLinearHeading(new Pose2d(-2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .addTemporalMarker(slides::hookPosDown)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{mode = Mode.SCORE;})
                .waitSeconds(0.5)
//                .addTemporalMarker(()->lastPos = cycle2.end())
                .addTemporalMarker(()->drive.followTrajectorySequenceAsync(cycle3))
                .build();

        cycle3 = drive.trajectorySequenceBuilder(cycle2.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(52, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62, -15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(62, -50), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -66), Math.toRadians(270))
                .addTemporalMarker(()->{mode = Mode.INTAKE;})
                .waitSeconds(0.5)
                .addTemporalMarker(slides::hookPosUp)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .lineToLinearHeading(new Pose2d(-2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .addTemporalMarker(slides::hookPosDown)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{mode = Mode.SCORE;})
                .waitSeconds(0.5)
//                .addTemporalMarker(()->lastPos = cycle3.end())
                .addTemporalMarker(()->drive.followTrajectorySequenceAsync(end))
                .build();

        end = drive.trajectorySequenceBuilder(cycle3.end())
                .lineToLinearHeading(new Pose2d(48, -66, Math.toRadians(270)))
                .addTemporalMarker(()->{mode = Mode.INTAKE;})
                .waitSeconds(0.5)
                .addTemporalMarker(slides::hookPosUp)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .lineToLinearHeading(new Pose2d(-2, -24-(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)))
                .addTemporalMarker(slides::hookPosDown)
                .addTemporalMarker(()->{mode = Mode.MOVING;})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{mode = Mode.SCORE;})
                .waitSeconds(0.5)
//                .addTemporalMarker(()->{mode = Mode.STOP;})
                .build();

        drive.followTrajectorySequenceAsync(entrance);

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && drive.isBusy()) {
            drive.update();
            slidePos = slideMotor.getCurrentPosition();
            targetPos = slideMotor.getTargetPosition();
            switch (mode){
                case MOVING:
                    telemetry.addData("Mode", "MOVING");
                    if(slidePos == targetPos){
                        mode = Mode.NULL;
                    }
                case INTAKE:
                    telemetry.addData("Mode", "INTAKE");
                    clawServo.setPosition(1);
                    time.reset();
                    while(time.seconds()<0.5 && opModeIsActive() && !isStopRequested()){
                        continue;
                    }
                    mode = Mode.NULL;
                case SCORE:
                    telemetry.addData("Mode", "SCORE");
                    clawServo.setPosition(0);
                    time.reset();
                    while(time.seconds()<0.5 && opModeIsActive() && !isStopRequested()){
                        continue;
                    }
                    mode = Mode.NULL;
                case NULL:
                    telemetry.addData("Mode", "NULL");
                    slides.stop();
                case STOP:
                    break;
            }
        }
        PositionStorage.pose = drive.getPoseEstimate();
    }
}
