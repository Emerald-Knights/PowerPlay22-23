package com.example.meepmeeptesting;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

//        RoadRunnerBotEntity bluebot1 = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
//                                .addDisplacementMarker(15, () -> {
//                                    //start moving slides
//                                })
//                                .lineToLinearHeading(new Pose2d(-32, 8, Math.toRadians(-45)))
//                                .forward(5)
//                                .addDisplacementMarker(() -> {
//                                    //drop thingy and start moving slide down
//                                })
//                                .back(10)
//                                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
//                                .addDisplacementMarker(() -> {
//                                    //pick up cone on top of stack and start moving slide up
//                                })
//                                .lineToLinearHeading(new Pose2d(-32, 16, Math.toRadians(45)))
//                                .forward(5)
//                                .back(10)
//                                .addDisplacementMarker(() -> {
//                                    //drop cone
//                                })
//                                .lineToSplineHeading(new Pose2d(-35, 35, Math.toRadians(0)))
//                                .build()
//                );
//
//        RoadRunnerBotEntity bluebot2 = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
//                                .addDisplacementMarker(15, () -> {
//                                    //start moving slides
//                                })
//                                .lineToLinearHeading(new Pose2d(-32, 8, Math.toRadians(-45)))
//                                .forward(5)
//                                .addDisplacementMarker(() -> {
//                                    //drop thingy and start moving slide down
//                                })
//                                .back(10)
//                                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
//                                .addDisplacementMarker(() -> {
//                                    //pick up cone on top of stack and start moving slide up
//                                })
//                                .lineToLinearHeading(new Pose2d(-32, 16, Math.toRadians(45)))
//                                .forward(5)
//                                .back(10)
//                                .addDisplacementMarker(() -> {
//                                    //drop cone
//                                })
//                                .lineToSplineHeading(new Pose2d(-35, 35, Math.toRadians(0)))
//                                .lineTo(new Vector2d(-12,35))
//                                .build()
//                );
//
//        RoadRunnerBotEntity bluebot3 = new DefaultBotBuilder(meepMeep)
//                .setColorScheme(new ColorSchemeBlueLight())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))
//                                .addDisplacementMarker(15, () -> {
//                                    //start moving slides
//                                })
//                                .lineToLinearHeading(new Pose2d(-32, 8, Math.toRadians(-45)))
//                                .forward(5)
//                                .addDisplacementMarker(() -> {
//                                    //drop thingy and start moving slide down
//                                })
//                                .back(10)
//                                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
//                                .addDisplacementMarker(() -> {
//                                    //pick up cone on top of stack and start moving slide up
//                                })
//                                .lineToLinearHeading(new Pose2d(-32, 16, Math.toRadians(45)))
//                                .forward(5)
//                                .back(10)
//                                .addDisplacementMarker(() -> {
//                                    //drop cone
//                                })
//                                .lineToSplineHeading(new Pose2d(-35, 35, Math.toRadians(0)))
//                                .lineTo(new Vector2d(-60,35))
//                                .build()
//                );

        int numCycles = 0;

        RoadRunnerBotEntity jack = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        //drive.trajectorySequenceBuilder(new Pose2d(36, 60, Math.toRadians(-90)))
                                drive.trajectorySequenceBuilder(new Pose2d(36, 15, Math.toRadians(-130)))
                                //goes from starting position and drops the cone and then parks in the left square
                                // .forward(45)
                               // .splineToLinearHeading(new Pose2d(30,8, Math.toRadians(-130)), Math.toRadians(0))
                               // .forward(8)
                               // .back(10)
                               // .lineToLinearHeading(new Pose2d(36,15, Math.toRadians(-270)))
                                //.forward(20.5)
                                //.lineToLinearHeading(new Pose2d(15,36, Math.toRadians(-270)))

                                //goes from starting position and drops the cone and then parks in the right square
                                //.forward(45)
                                //.splineToLinearHeading(new Pose2d(30,8, Math.toRadians(-130)), Math.toRadians(0))
                                //.forward(8)
                                //.back(10)
                                //.lineToLinearHeading(new Pose2d(36,15, Math.toRadians(-270)))
                                //.forward(20.5)
                                //.lineToLinearHeading(new Pose2d(60,34.5, Math.toRadians(-270)))

                                //goes from starting position and drops the cone and then parks in the middle square
                                //.forward(45)
                                //.lineToLinearHeading(new Pose2d(30,8, Math.toRadians(-130)))
//                                .forward(8)
//                                .back(10)
//                                .lineToLinearHeading(new Pose2d(36,15, Math.toRadians(-270)))
//                                .forward(20.5)

                                /*.forward(0.1)
                                .lineToLinearHeading(new Pose2d(36,10, Math.toRadians(0)))
                                        .strafeLeft(2.3)
                                        .forward(25)
                                        .back(25)
                                        .strafeRight(2.3)
                                        .lineToLinearHeading(new Pose2d(36, 15, Math.toRadians(-130)))
                                        .forward(15)
                                        .back(15)*/

                                        .forward(0.1)
                                        .lineToLinearHeading(new Pose2d(36,10, Math.toRadians(0)))
                                        .strafeLeft(2.3)
                                        .forward(25)
                                        .back(25)
                                        .strafeRight(2.3)
                                        .lineToLinearHeading(new Pose2d(36, 11, Math.toRadians(-180)))
                                        .strafeTo(new Vector2d(11,13.1))
                                        .lineToLinearHeading(new Pose2d(11,13, Math.toRadians(-225)))
                                        .forward(15)
                                        .back(15)


                                    .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(jack)
                .start();
    }
}