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

public class TigerMeepMeepTestingV2 {
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
                .setConstraints(64, 64, Math.toRadians(60), 4.467498059104943, trackWidth)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(4.77, -69.93, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(5.4, -34.65, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(36, -13 , Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(46,-10, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(47, -57, Math.toRadians(-90)))
                                //close claw
                                //slideup
                                .lineToSplineHeading(new Pose2d(5.4, -34.65, Math.toRadians(90)))
                                //hook and lower slide
                                .lineToSplineHeading(new Pose2d(47, -57, Math.toRadians(-90)))
                                //close claw
                                //slide up
                                .lineToSplineHeading(new Pose2d(5.4, -34.65, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(47,-10, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(54,-10, Math.toRadians(90)))
                                .back(45)
                                //hook and lower claw finish
                                /*.splineTo(new Vector2d(32.74, -34.81), Math.toRadians(-0.33))
                                .splineTo(new Vector2d(47.05, -0.16), Math.toRadians(67.57))
                                .splineTo(new Vector2d(58.33, -0.16), Math.toRadians(0.00))
                                .splineTo(new Vector2d(61.03, -23.52), Math.toRadians(258.39))
                                .splineTo(new Vector2d(59.92, -67.07), Math.toRadians(-88.64))*/

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