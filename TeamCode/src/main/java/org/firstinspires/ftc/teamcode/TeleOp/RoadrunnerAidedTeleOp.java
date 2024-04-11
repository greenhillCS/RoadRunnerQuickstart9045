package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group="test", name="RoadrunnerAidedTeleOp")
public class RoadrunnerAidedTeleOp extends LinearOpMode {
    @Override
    public void runOpMode (){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        Trajectory originTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(0, 0, 0))
                .build();

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.x){
                drive.followTrajectory(originTrajectory);
            }else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
