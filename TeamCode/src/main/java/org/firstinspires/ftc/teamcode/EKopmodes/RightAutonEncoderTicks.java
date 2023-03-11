package org.firstinspires.ftc.teamcode.EKopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;

@Autonomous(name="RightAutonEncoderTicks", group="auto")
public class RightAutonEncoderTicks extends LinearOpMode {

    Robot wucru;

    @Override
    public void runOpMode() throws InterruptedException {
        wucru = new Robot(hardwareMap, this);
        wucru.initOpenCV();
        waitForStart();

        //start sequence
        wucru.straightOneOdo(1,0.2,48,true);

    }
}