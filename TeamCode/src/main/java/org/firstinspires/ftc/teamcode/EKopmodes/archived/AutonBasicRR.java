package org.firstinspires.ftc.teamcode.EKopmodes.archived;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="W-AUTON", group="auto")
public class AutonBasicRR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        Robot wucru = new Robot(hardwareMap, this);

        //wucru.initOpenCV();
        waitForStart();
        wucru.resetEncoders();

        Trajectory traj_left = wucru.trajectoryBuilder(new Pose2d(), false)
                .strafeLeft(24)
                .build();

        Trajectory traj_leftToPole = wucru.trajectoryBuilder(traj_left.end(), false)
                .splineToLinearHeading(new Pose2d(31, 18, Math.toRadians(-45)), Math.toRadians(0))
                .build();

        Trajectory traj_adjustToPole = wucru.trajectoryBuilder(traj_leftToPole.end(), false)
                .forward(6)
                .build();

        Trajectory traj_backBeforePark = wucru.trajectoryBuilder(traj_adjustToPole.end(), false)
                .back(12)
                .build();

        Trajectory traj_park;
        if(DetectorPipeline.sleeveColor == 0) {
            traj_park = wucru.trajectoryBuilder(traj_backBeforePark.end(), false)
                    .lineToLinearHeading(new Pose2d(23, -26, Math.toRadians(0)))
                    .build();
        } else if(DetectorPipeline.sleeveColor == 1) {
            traj_park = wucru.trajectoryBuilder(traj_backBeforePark.end(), false)
                    .lineToLinearHeading(new Pose2d(23, -2, Math.toRadians(0)))
                    .build();
        } else {
            traj_park = wucru.trajectoryBuilder(traj_backBeforePark.end(), false)
                    .lineToLinearHeading(new Pose2d(23, 22, Math.toRadians(0)))
                    .build();
        }

        wucru.moveClaw();
        sleep(1000);
        wucru.moveClaw();
        wucru.followTrajectory(traj_left);
        wucru.followTrajectory(traj_leftToPole);

        wucru.moveSlide(0.45, 7);
        wucru.followTrajectory(traj_adjustToPole);
        wucru.moveClaw();
        wucru.followTrajectory(traj_backBeforePark);

        wucru.followTrajectory(traj_park);
        wucru.moveSlide(-0.4, 7);
        sleep(1000);
    }
}
