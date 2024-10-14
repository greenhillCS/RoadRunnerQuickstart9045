package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

// IMPORTANT: Place robot in the center of 4 field tiles arranged in a square shape on start

// Optional: Have laptop open to see where the robot is when testing autonomous movement

@Autonomous(group="Test", name="RobotFunctionTest")
public class RobotFunctionTest extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor hangingMotor = hardwareMap.get(DcMotor.class, "HM");
        DcMotor scoringMotor = hardwareMap.get(DcMotor.class, "SM");

        IntoTheDeepSlides hangar = new IntoTheDeepSlides(hangingMotor);
        IntoTheDeepSlides scorer = new IntoTheDeepSlides(scoringMotor);

        Servo claw = hardwareMap.get(Servo.class, "CS");

        TrajectorySequence testTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(12, 12))
                .lineToSplineHeading(new Pose2d(12, -12, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-12, 12, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(12, 12, Math.toRadians(0)))
                .lineTo(new Vector2d(0, 0))
                .build();

        waitForStart();

        while (opModeIsActive()){
            for (int i = 0;i<1;i++) {
                claw.setPosition(0);
                claw.setPosition(1);
            }
            for (int i = 0;i<1;i++) {
                scorer.hookPosUp();
                while (scoringMotor.isBusy() & scorer.runTime.seconds() < scorer.timeOutSecs) {
                    telemetry.addData("Running: ", "Scoring Slides");
                    telemetry.update();
                }
                scorer.startPos();
                while (scoringMotor.isBusy() & scorer.runTime.seconds() < scorer.timeOutSecs) {
                    telemetry.addData("Running: ", "Scoring Slides");
                    telemetry.update();
                }
            }
            for (int i = 0;i<1;i++) {
                hangar.hookPosUp();
                while (scoringMotor.isBusy() & scorer.runTime.seconds() < scorer.timeOutSecs) {
                    telemetry.addData("Running: ", "Hanging Slides");
                    telemetry.update();
                }
                hangar.startPos();
                while (scoringMotor.isBusy() & scorer.runTime.seconds() < scorer.timeOutSecs) {
                    telemetry.addData("Running: ", "Hanging Slides");
                    telemetry.update();
                }
            }
            drive.followTrajectorySequence(testTrajectory);
            break;
        }

    }
}
