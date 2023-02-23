package org.firstinspires.ftc.teamcode.EKopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;

@Autonomous(name="LeftAuton", group="auto")
public class LeftAuton extends LinearOpMode {

    int numCycle = 1;
    int downEachStack = 5;
    Robot wucru;

    @Override
    public void runOpMode() throws InterruptedException{
        wucru = new Robot(hardwareMap, this);

        wucru.initOpenCV();
        waitForStart();

        Trajectory traj_start = wucru.trajectoryBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                //setup for cycle + initial drop
                .addDisplacementMarker(() -> {
                    wucru.moveClaw();
                    wucru.setSlidePosition(1);
                })
                .addTemporalMarker(2, () -> {
                    wucru.setSlidePosition(wucru.slidePosition[3]);
                })
                .lineToLinearHeading(new Pose2d(-36,-11,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-23.5,-11,Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    wucru.moveClaw();
                })
                .build();

        Trajectory traj_park;
        if(DetectorPipeline.sleeveColor == 1) {
            traj_park = wucru.trajectoryBuilder(new Pose2d(-23.5, -11, Math.toRadians(90)))
                    .addDisplacementMarker(() -> {
                        wucru.setSlidePosition(wucru.slidePosition[0]);
                    })
                    .lineToLinearHeading(new Pose2d(-12,-12,Math.toRadians(180)))
                    .build();
        } else if (DetectorPipeline.sleeveColor == 2) {
            traj_park = wucru.trajectoryBuilder(new Pose2d(-23.5, -11, Math.toRadians(90)))
                    .addDisplacementMarker(() -> {
                        wucru.setSlidePosition(wucru.slidePosition[0]);
                    })
                    .lineToLinearHeading(new Pose2d(-35,-12,Math.toRadians(180)))
                    .build();
        } else {
            traj_park = wucru.trajectoryBuilder(new Pose2d(-23.5, -11, Math.toRadians(90)))
                    .addDisplacementMarker(() -> {
                        wucru.setSlidePosition(wucru.slidePosition[0]);
                    })
                    .lineToLinearHeading(new Pose2d(-58,-12,Math.toRadians(180)))
                    .build();
        }

        wucru.followTrajectory(traj_start);
        for(int i = 0; i < numCycle; i++) {
            wucru.followTrajectory(buildCycle(wucru.slidePosition[1] - i * downEachStack));
        }
        wucru.followTrajectory(traj_park);
    }

    private Trajectory buildCycle(int slideHeight) {
        Trajectory traj_cycle = wucru.trajectoryBuilder(new Pose2d(-23.5, 11, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    wucru.setSlidePosition(slideHeight);
                })
                .lineToLinearHeading(new Pose2d(-55,-12,Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    //wucru.localize();
                })
                .lineToLinearHeading(new Pose2d(-58,-12,Math.toRadians(180)))//small move forward to get cone
                .addDisplacementMarker(() -> {
                    wucru.moveClaw(); //close
                    wucru.setSlidePosition(wucru.slidePosition[3]);
                })
                .lineToLinearHeading(new Pose2d(-23.5,-11,Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    wucru.moveClaw();
                })
                .build();
        return traj_cycle;
    }
}
