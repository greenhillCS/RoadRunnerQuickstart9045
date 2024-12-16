package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class UltimateJuicyAutonMeepMeep {

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
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(64, 64, Math.toRadians(60), 4.467498059104943, trackWidth)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(4.77, -69.93, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(32, -41, Math.toRadians(42)))
                                //grab 1
                                .turn(Math.toRadians(-115))
                                //drop 1
                                .turn(Math.toRadians(105))
                                //grab 2
                                .turn(Math.toRadians(-105))
                                //drop 2
                                .turn(Math.toRadians(95))
                                //grab 3
                                .turn(Math.toRadians(-95))
                                //drop 3
                                .lineToSplineHeading(new Pose2d(26, -41, Math.toRadians(90)))
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
