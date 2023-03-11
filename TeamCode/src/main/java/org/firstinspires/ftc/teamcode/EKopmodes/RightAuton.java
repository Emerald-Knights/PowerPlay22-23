package org.firstinspires.ftc.teamcode.EKopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;

@Autonomous(name="RightAuton", group="auto")
public class RightAuton extends LinearOpMode {

    int numCycle = 1;
    int downEachStack = 5;
    Robot wucru;

    @Override
    public void runOpMode() throws InterruptedException{
        wucru = new Robot(hardwareMap, this);

//        wucru.initOpenCV();
        waitForStart();

//        wucru.setPoseEstimate(new Pose2d(-36, 60, Math.toRadians(-90)));

        Trajectory traj_start1 = wucru.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(50.5,-5 ,Math.toRadians(45)))
                .build();


        Trajectory traj_start2= wucru.trajectoryBuilder(traj_start1.end())
                .forward(16)
                .build();

        Trajectory traj_start3 = wucru.trajectoryBuilder(traj_start2.end()).back(14).build();

        Trajectory turn_to_cone = wucru.trajectoryBuilder(traj_start3.end())
                .lineToLinearHeading(new Pose2d(48,-10, Math.toRadians(90))).build();

        Trajectory coneForward = wucru.trajectoryBuilder(turn_to_cone.end()).back(10).build();

        Trajectory coneBackward = wucru.trajectoryBuilder(coneForward.end()).forward(10).build();

        Trajectory back_to_junction = wucru.trajectoryBuilder(coneBackward.end())
                .lineToLinearHeading(new Pose2d(50,0, Math.toRadians(45))).build();

        Trajectory junctionForw = wucru.trajectoryBuilder(back_to_junction.end())
                .forward(16).build();

//        Trajectory traj_park;
//        if(DetectorPipeline.sleeveColor == 1) {
//            traj_park = wucru.trajectoryBuilder(new Pose2d(-23.5, 11, Math.toRadians(-90)))
//                    .addDisplacementMarker(() -> {
//                        wucru.setSlidePosition(wucru.slidePosition[0]);
//                    })
//                    .lineToLinearHeading(new Pose2d(-12,12,Math.toRadians(180)))
//                    .build();
//        } else if (DetectorPipeline.sleeveColor == 2) {
//            traj_park = wucru.trajectoryBuilder(new Pose2d(-23.5, 11, Math.toRadians(-90)))
//                    .addDisplacementMarker(() -> {
//                        wucru.setSlidePosition(wucru.slidePosition[0]);
//                    })
//                    .lineToLinearHeading(new Pose2d(-35,12,Math.toRadians(180)))
//                    .build();
//        } else {
//            traj_park = wucru.trajectoryBuilder(new Pose2d(-23.5, 11, Math.toRadians(-90)))
//                    .addDisplacementMarker(() -> {
//                        wucru.setSlidePosition(wucru.slidePosition[0]);
//                    })
//                    .lineToLinearHeading(new Pose2d(-58,12,Math.toRadians(180)))
//                    .build();
//        }

        wucru.moveClaw();
        wucru.neck.setPosition(0.15);
        wucru.followTrajectory(traj_start1);
        wucru.neck.setPosition(0.15);
        wucru.moveClaw();
        wucru.followTrajectory(traj_start2);
        wucru.followTrajectory(traj_start3);


        //INSERT OPENCV LOCALIZATION?

        wucru.followTrajectory(turn_to_cone);
        wucru.neck.setPosition(1);
        wucru.followTrajectory(coneForward);
        wucru.moveClaw();
        wucru.followTrajectory(coneBackward);
        wucru.followTrajectory(back_to_junction);
        wucru.followTrajectory(junctionForw);


//        for(int i = 0; i < numCycle; i++) {
//            wucru.followTrajectory(buildCycle(wucru.slidePosition[1] - i * downEachStack));
//        }
//        wucru.followTrajectory(traj_park);
    }

    private Trajectory buildCycle(int slideHeight) {
        Trajectory traj_cycle = wucru.trajectoryBuilder(new Pose2d(-23.5, 11, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    wucru.setSlidePosition(slideHeight);
                })
                .lineToLinearHeading(new Pose2d(-55,12,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    //wucru.localize();
                })
                .lineToLinearHeading(new Pose2d(-58,12,Math.toRadians(180)))//small move forward to get cone
                .addDisplacementMarker(() -> {
                    wucru.moveClaw(); //close
                    wucru.setSlidePosition(wucru.slidePosition[3]);
                })
                .lineToLinearHeading(new Pose2d(-23.5,11,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    wucru.moveClaw();
                })
                .build();
        return traj_cycle;
    }
}
