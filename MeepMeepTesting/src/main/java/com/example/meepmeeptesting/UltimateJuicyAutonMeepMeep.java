package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        double trackWidth = 18.63;

        double botLength = 16.33858;

        double[] startingPos = {-36, 68-(botLength/2)};
        double startHeading = Math.toRadians(90);

        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(64, 64, 4.9, Math.toRadians(360), trackWidth)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -72+(botLength/2), Math.toRadians(-90.00)))
                                .setReversed(true)
                                .splineTo(new Vector2d(5, -34), Math.toRadians(90))
                                //start clip
                                .lineToSplineHeading(new Pose2d(44, -41, Math.toRadians(75)))
                                //grab 1
                                .turn(Math.toRadians(-145)) //15
                                //drop 1
                                .turn(Math.toRadians(117)) //45
                                //grab 2
                                .turn(Math.toRadians(-117))
                                //drop 2
                                .turn(Math.toRadians(105))
                                //grab 3
                                .turn(Math.toRadians(-105))
                                //drop 3
                                .lineToLinearHeading(new Pose2d(48, -58, Math.toRadians(-90)))
                                .lineTo(new Vector2d(4, -48))
                                .lineTo(new Vector2d(48, -58))
                                .lineTo(new Vector2d(2, -48))
                                .lineTo(new Vector2d(48, -58))
                                .lineTo(new Vector2d(0, -48))
                                .lineTo(new Vector2d(48, -58))
                                .lineTo(new Vector2d(-2, -48))
                                .lineTo(new Vector2d(48, -58))
                                .lineTo(new Vector2d(-4, -48))
                                .lineToSplineHeading(new Pose2d(44, -60, Math.toRadians(90)))
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
