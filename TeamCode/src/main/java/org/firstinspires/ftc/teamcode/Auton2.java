//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
//
//@Autonomous(name="W-AUTON-LEFT", group="auto")
//public class Auton2 extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException{
//
//        Robot wucru = new Robot(hardwareMap, this);
//
//        wucru.initOpenCV();
//        waitForStart();
//        wucru.resetEncoders();
//
//        Trajectory traj_left = wucru.trajectoryBuilder(new Pose2d(), false)
//                .strafeRight(30)
//                .build();
//
//        Trajectory traj_leftToPole = wucru.trajectoryBuilder(traj_left.end(), false)
//                .splineToLinearHeading(new Pose2d(31, -23, Math.toRadians(45)), Math.toRadians(0))
//                .build();
//
//        Trajectory traj_adjustToPole = wucru.trajectoryBuilder(traj_leftToPole.end(), false)
//                .forward(9)
//                .build();
//
//        Trajectory traj_backBeforePark = wucru.trajectoryBuilder(traj_adjustToPole.end(), false)
//                .back(12)
//                .build();
//
//        Trajectory traj_park;
//        telemetry.addData("sleeveColor", DetectorPipeline.sleeveColor);
//        telemetry.update();
//        if(DetectorPipeline.sleeveColor == 3) {
//            traj_park = wucru.trajectoryBuilder(traj_backBeforePark.end(), false)
//                    .lineToLinearHeading(new Pose2d(23, -27, Math.toRadians(0)))
//                    .build();
//        } else if(DetectorPipeline.sleeveColor == 2) {
//            traj_park = wucru.trajectoryBuilder(traj_backBeforePark.end(), false)
//                    .lineToLinearHeading(new Pose2d(23, -2, Math.toRadians(0)))
//                    .build();
//        } else {
//            traj_park = wucru.trajectoryBuilder(traj_backBeforePark.end(), false)
//                    .lineToLinearHeading(new Pose2d(23, 22, Math.toRadians(0)))
//                    .build();
//        }
//
////        wucru.moveClaw();
////        sleep(1000);
////        wucru.moveClaw();
////        wucru.followTrajectory(traj_left);
////        wucru.followTrajectory(traj_leftToPole);
////
////        wucru.moveSlide(0.45, 7);
////        wucru.followTrajectory(traj_adjustToPole);
////        wucru.moveSlide(-0.45, 1.5);
////        wucru.moveClaw();
////        wucru.moveSlide(0.5, 1.5);
////        wucru.followTrajectory(traj_backBeforePark);
////
////        wucru.followTrajectory(traj_park);
////        wucru.moveSlide(-0.4, 7);
////        sleep(1000);
//    }
//}
