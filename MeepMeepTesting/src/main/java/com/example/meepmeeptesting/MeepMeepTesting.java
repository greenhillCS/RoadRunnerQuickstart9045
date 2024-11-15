package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\sewakj27\\StudioProjects\\RoadRunnerQuickstart9045\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\field-2024-juice-dark.png")); }
        catch (IOException e) {}
        double trackWidth = 17.785;

        double botLength = 17.323;

        double[] startingPos = {-12, 72-(botLength/2)};
        double startHeading = Math.toRadians(-90);

        double pushDist = -57;
        double baseDist = -8.5;

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(64, 64, Math.toRadians(90), 4.467498059104943, trackWidth)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -72+(botLength/2), Math.toRadians(90.00)))
                                .splineToConstantHeading(new Vector2d(0, -26-(botLength/2)), Math.toRadians(90))

                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(36, -36, Math.toRadians(270)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(36, -15), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(46, -15), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(48, -58), Math.toRadians(270))
                                .lineToLinearHeading(new Pose2d(0, -26-(botLength/2), Math.toRadians(90)))

                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(46, -36, Math.toRadians(270)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(46, -15), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(56, -15), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(48, -58), Math.toRadians(270))
                                .lineToLinearHeading(new Pose2d(0, -26-(botLength/2), Math.toRadians(90)))

                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(52, -36, Math.toRadians(270)), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(52, -15), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(62, -15), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(62, -50), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(48, -58), Math.toRadians(270))
                                .lineToLinearHeading(new Pose2d(0, -26-(botLength/2), Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(48, -58, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(0, -26-(botLength/2), Math.toRadians(90)))
                                .build()


                );

        // Set field image
        meepMeep.setBackground(img)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}