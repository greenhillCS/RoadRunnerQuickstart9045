package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Vision {
    VisionPortal visionPortal;
    AprilTagProcessor tagProcessor;
    public Vision(VisionPortal v, AprilTagProcessor t){
        visionPortal = v;
        tagProcessor = t;
    }
    public ArrayList<Pose2d> getAprilTagPoses() {
        ArrayList<AprilTagDetection> tags = new ArrayList<>();

        if (visionPortal.getProcessorEnabled(tagProcessor)) {
            tags = tagProcessor.getDetections();
        }

        ArrayList<Pose2d> poses = new ArrayList<>();

        for (AprilTagDetection tag: tags) {
            if (tag.metadata != null) {

                // Get the tag absolute position on the field
                Transform3d tagPose = new Transform3d(
                        tag.metadata.fieldPosition,
                        tag.metadata.fieldOrientation
                );

                // Get the relative location of the tag from the camera
                Transform3d cameraToTagTransform = new Transform3d(
                        new VectorF(
                                (float) tag.rawPose.x,
                                (float) tag.rawPose.y,
                                (float) tag.rawPose.z
                        ),
                        Transform3d.MatrixToQuaternion(tag.rawPose.R)
                );

                // Inverse the previous transform to get the location of the camera from the tag
                Transform3d tagToCameraTransform = cameraToTagTransform.unaryMinusInverse();

                // Add the tag position and the relative position of the camera to the tag
                Transform3d cameraPose = tagPose.plus(tagToCameraTransform);

                // The the relative location of the camera to the robot
                Transform3d robotToCameraTransform = new Transform3d(
                        new VectorF(
                                0.0f,
                                0.0f,
                                0.0f
                        ),
                        new Quaternion(0,0,1f,0, System.nanoTime())
                );

                // Inverse the previous transform again to get the location of the robot from the camera
                Transform3d cameraToRobotTransform = robotToCameraTransform.unaryMinusInverse();

                // Add the relative location of the robot to location of the Camera
                Transform3d robotPose = cameraPose.plus(cameraToRobotTransform);

                // Convert from a 3D transform to a 2D pose
                poses.add(robotPose.toPose2d());
            }
        }

        return poses;
    }

}
