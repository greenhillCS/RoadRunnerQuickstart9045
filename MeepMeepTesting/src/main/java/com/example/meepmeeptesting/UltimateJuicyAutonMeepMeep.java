package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import java.nio.file.Path;

import javax.imageio.ImageIO;

public class UltimateJuicyAutonMeepMeep {

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        Image img = null;
        String classPath = UltimateJuicyAutonMeepMeep.class.getProtectionDomain().getCodeSource().getLocation().getPath();
        File classFile = new File(classPath);
        File parentDirectory = classFile.getParentFile().getParentFile().getParentFile().getParentFile().getParentFile();
        System.out.print(parentDirectory+"\\field-2024-juice-dark.png");
        try { img = ImageIO.read(new File(parentDirectory+"\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\field-2024-juice-dark.png")); }
        catch (IOException e) {}
        double trackWidth = 16.33858;

        double botLength = 16.33858;

        double[] startingPos = {-36, 68-(botLength/2)};
        double startHeading = Math.toRadians(-90);

        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(64, 64, Math.toRadians(60), 4.467498059104943, trackWidth)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -72+(botLength/2), Math.toRadians(-90.00)))
                                .lineToSplineHeading(new Pose2d(32, -41, Math.toRadians(-135)))
                                //grab 1
                                .turn(Math.toRadians(-90))
                                //drop 1
                                .turn(Math.toRadians(70))
                                //grab 2
                                .turn(Math.toRadians(-70))
                                //drop 2
                                .turn(Math.toRadians(65))
                                //grab 3
                                .turn(Math.toRadians(-65))
                                //drop 3
                                .lineToSplineHeading(new Pose2d(26, -41, Math.toRadians(125)))
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
