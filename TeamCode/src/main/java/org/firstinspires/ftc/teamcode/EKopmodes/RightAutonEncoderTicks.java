package org.firstinspires.ftc.teamcode.EKopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;

@Autonomous(name="RightAutonEncoderTicks", group="auto")
public class RightAutonEncoderTicks extends LinearOpMode {

    Robot wucru;

    @Override
    public void runOpMode() throws InterruptedException {
        wucru = new Robot(hardwareMap, this);
//        wucru.initOpenCV();
        waitForStart();


        //start sequence
        wucru.moveClaw();
        wucru.straightOneOdo(1,0.4,50, 0, 0.03, true);
        wucru.turnTo(Math.PI/4, 0.4, -1);
        wucru.moveClaw();

        //deposit preloaded cone
        sleep(1000);
        wucru.moveSlide(10, 0.4);
//        sleep(2000);


        //this line is not working for some reason
        wucru.straightOneOdo(1, 0.3, 8, Math.PI/4, 0, true);

//        wucru.straightWtime(1, 0.15, 1);
        sleep(2000);
        wucru.straightOneOdo(-1, 0.3, 8, Math.PI/4,0, true);
        sleep(1000);

        //turn toward cone stack
        wucru.turnTo(Math.PI/2, 0.15, -1);
//        sleep(2000);
//
//        //for loop starts
//        //insert camera localization sequence
//        wucru.straightOneOdo(-1, 0.4, 14, Math.PI/2, 0.03, true);
//        wucru.moveSlide(5, -0.4);





        wucru.moveClaw();


        telemetry.update();

    }
}