package com.example.meepmeeptesting;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity blueStart = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
                                //setup for cycle + initial drop
                                .lineToLinearHeading(new Pose2d(-36,11,Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-23.5,11,Math.toRadians(-90)))

                                .build()
                );

        RoadRunnerBotEntity blueCycle = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23.5, 11, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-55,12,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-58,12,Math.toRadians(180)))//small move forward to get cone
                                .lineToLinearHeading(new Pose2d(-23.5,11,Math.toRadians(-90)))

                                .build()
                );
        RoadRunnerBotEntity blueParkOne = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23.5, 11, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-12,12,Math.toRadians(180)))
                                .build()
                );
        RoadRunnerBotEntity blueParkTwo = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23.5, 11, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-35,12,Math.toRadians(180)))
                                .build()
                );
        RoadRunnerBotEntity blueParkThree = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23.5, 11, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-58,12,Math.toRadians(180)))
                                .build()
                );

        RoadRunnerBotEntity redStart = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                //setup for cycle + initial drop
                                .lineToLinearHeading(new Pose2d(-36,-11,Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-23.5,-11,Math.toRadians(90)))

                                .build()
                );

        RoadRunnerBotEntity redCycle = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23.5, -11, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-55,-12,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-58,-12,Math.toRadians(180)))//small move forward to get cone
                                .lineToLinearHeading(new Pose2d(-23.5,-11,Math.toRadians(90)))

                                .build()
                );
        RoadRunnerBotEntity redParkOne = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23.5, -11, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(180)))
                                .build()
                );
        RoadRunnerBotEntity redParkTwo = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23.5, -11, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-35,-12,Math.toRadians(180)))
                                .build()
                );
        RoadRunnerBotEntity redParkThree = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-23.5, -11, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-58,-12,Math.toRadians(180)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(blueParkOne)
                .addEntity(redParkOne)



                .start();
    }
}
