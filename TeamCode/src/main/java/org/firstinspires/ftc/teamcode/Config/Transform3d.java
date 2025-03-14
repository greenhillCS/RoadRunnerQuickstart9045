package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

//import javax.annotation.processing.AbstractProcessor;

/**
 * Class for transformations in 3D Space. Only uses libraries in the FTC SDK,
 * with the optional addition of RoadRunner for ease of use with relocalization.
 */
public class Transform3d {

    /**
     * Vector to describe a 3-dimensional position in space
     */
    VectorF translation;
    /**
     * Quaternion to describe a 3-dimensional orientation in space
     */
    Quaternion rotation;

    private final static VectorF zeroVector = new VectorF(0,0,0);

    public Transform3d() {
        translation = new VectorF(0f,0f,0f);
        rotation = new Quaternion();
    }

    public Transform3d(VectorF t, Quaternion Q) {
        translation = t;
        rotation = Q;
    }

    public Transform3d unaryMinusInverse() {
        Quaternion Q = rotation.inverse();
        VectorF t = rotation.applyToVector(zeroVector.subtracted(translation));
        return new Transform3d(t,Q);
    }

    public Transform3d plus(Transform3d P) {
        VectorF t = translation.added(rotation.applyToVector(P.translation));
        Quaternion Q = rotation.multiply(P.rotation, System.nanoTime());
        return new Transform3d(t,Q);
    }

    public double getZ() {
        return Math.atan2(2.0*(rotation.y*rotation.z + rotation.w*rotation.x),
                rotation.w*rotation.w - rotation.x*rotation.x - rotation.y*rotation.y + rotation.z*rotation.z);
    }

    public static Quaternion MatrixToQuaternion(MatrixF m1) {
        float w = (float) (Math.sqrt(1.0 + m1.get(0,0) + m1.get(1,1) + m1.get(2,2)) / 2.0);
        float w4 = (float) (4.0 * w);
        float x = (m1.get(2,1) - m1.get(1,2)) / w4 ;
        float y = (m1.get(0,2) - m1.get(2,0)) / w4 ;
        float z = (m1.get(1,0) - m1.get(0,1)) / w4 ;
        return new Quaternion(w,x,y,z, System.nanoTime());
    }

    public Pose2d toPose2d() {
        return new Pose2d(
                translation.get(0),
                translation.get(1),
                this.getZ()
        );
    }
}

