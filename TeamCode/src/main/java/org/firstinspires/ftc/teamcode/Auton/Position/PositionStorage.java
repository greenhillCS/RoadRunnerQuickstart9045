package org.firstinspires.ftc.teamcode.Auton.Position;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;

public class PositionStorage {
    public static Pose2d pose = new Pose2d(0, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90));
}
