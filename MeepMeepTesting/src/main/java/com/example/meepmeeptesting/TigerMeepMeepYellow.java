package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class TigerMeepMeepYellow {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\ZhaoT28\\StudioProjects\\RoadRunnerQuickstart9045Tiger\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\field-2024-juice-dark.png")); }
        catch (IOException e) {}
        double trackWidth = 16.33858;

        double botLength = 16.33858;

        double[] startingPos = {-36, 68-(botLength/2)};
        double startHeading = Math.toRadians(-90);

        MeepMeep meepMeep = new MeepMeep(800);
        /*.lineToSplineHeading(new Pose2d(49, 36, Math.toRadians(180)))
                                //drop arm
                                .lineToConstantHeading(new Vector2d(0, 0))
                                .lineToConstantHeading(new Vector2d(-59, 23.5))
                                */

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(60), 4.467498059104943, trackWidth)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-19, -60, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-60, -60,  Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-38.35, -32.22,  Math.toRadians(135)))
                                .lineToSplineHeading(new Pose2d(-44, -28, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-60, -60, Math.toRadians(-135)))
                                .lineToSplineHeading(new Pose2d(-44, -28, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-52, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-60, -65, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-54, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-60.5, -12, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(-61, -66, Math.toRadians(180)))
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