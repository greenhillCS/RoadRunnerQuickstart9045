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

public class TejExceelectAmaazingSqaureAutonomweous {

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        Image img = null;
        String classPath = TejExceelectAmaazingSqaureAutonomweous.class.getProtectionDomain().getCodeSource().getLocation().getPath();
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
                        drive.trajectorySequenceBuilder(new Pose2d(56, -53, Math.toRadians(90.00)))
                                .setReversed(true)
                                .lineTo(new Vector2d(53,53))
                                .turn(Math.toRadians(90))

                                .lineTo(new Vector2d(-53,53))
                                .turn(Math.toRadians(90))

                                .lineTo(new Vector2d(-53,-53))
                                .turn(Math.toRadians(90))

                                .lineTo(new Vector2d(53,-53))
                                .turn(Math.toRadians(90))
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
