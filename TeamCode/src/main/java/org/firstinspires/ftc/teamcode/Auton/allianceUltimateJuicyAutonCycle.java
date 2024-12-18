package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group="intothedeep", name="ultimate claw cycle")
public class allianceUltimateJuicyAutonCycle extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo clawServo = hardwareMap.get(Servo.class, "CS");

        RevTouchSensor scoringTouchSensor = hardwareMap.get(RevTouchSensor.class, "TS");

        IntoTheDeepSlides slides = new IntoTheDeepSlides(slideMotor, telemetry, scoringTouchSensor);



        drive.setPoseEstimate(new Pose2d(12, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90.00)));

        clawServo.setPosition(1);
        slides.moveToWait(-10);
        while (slideMotor.isBusy() && slides.runTime.seconds() < slides.timeOutSecs && opModeIsActive() && !isStopRequested()) {
            continue;
        }


        TrajectorySequence t1 =  drive.trajectorySequenceBuilder(new Pose2d(4.77, -69.93, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(32, -41, Math.toRadians(42)))
                //grab 1
                .turn(Math.toRadians(-115))
                //drop 1
                .turn(Math.toRadians(105))
                //grab 2
                .turn(Math.toRadians(-105))
                //drop 2
                .turn(Math.toRadians(95))
                //grab 3
                .turn(Math.toRadians(-95))
                //drop 3
                .lineToSplineHeading(new Pose2d(26, -41, Math.toRadians(90)))
                .build();

        waitForStart();
        drive.followTrajectorySequence(t1);

    }
}
