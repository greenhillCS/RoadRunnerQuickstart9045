package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        double trackWidth = 16.33858;

        double botLength = 16.33858;

        double[] startingPos = {-36, 70-(botLength/2)};
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
                        drive.trajectorySequenceBuilder(new Pose2d(startingPos[0], startingPos[1], startHeading))
                                .lineTo(new Vector2d(startingPos[0], startingPos[1] - 27))
                                .turn(Math.toRadians(360))
                                .lineToSplineHeading(new Pose2d(15, startingPos[1] - 27, Math.toRadians(-90)))
                                .lineToSplineHeading(new Pose2d(48, startingPos[1] - 27, 0))
                                .lineToSplineHeading(new Pose2d(0, startingPos[1] - 27, Math.toRadians(180)))
                                .lineTo(new Vector2d(-61, startingPos[1] - 27))
                                .lineTo(new Vector2d(10, startingPos[1] - 27))
                                .lineToSplineHeading(new Pose2d(48, startingPos[1] - 27, 0))
                                .lineToSplineHeading(new Pose2d(48, startingPos[1] - 3, 0))
                                .lineToSplineHeading(new Pose2d(60, startingPos[1] - 3, 0))
                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}