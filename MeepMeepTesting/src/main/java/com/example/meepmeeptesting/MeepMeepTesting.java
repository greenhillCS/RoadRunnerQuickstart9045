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

        double pushDist = 55;

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(60), 4.467498059104943, trackWidth)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 72-(trackWidth/2), Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(0, 24+(trackWidth/2), Math.toRadians(90)))
                                // Raise slides to hang specimen
                                .lineToSplineHeading(new Pose2d(36, 30+(trackWidth/2), Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(40, 12, Math.toRadians(68.7)))
                                .lineTo(new Vector2d(58, 47))
                                .lineTo(new Vector2d(46, pushDist))
                                .lineTo(new Vector2d(46, 12))
                                .lineTo(new Vector2d(56, 12))
                                .lineTo(new Vector2d(56, pushDist))
                                .lineTo(new Vector2d(56, 12))
                                .lineTo(new Vector2d(61, 12))
                                .lineTo(new Vector2d(61, pushDist))
                                .lineToSplineHeading(new Pose2d(12, 0, Math.toRadians(180)))
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