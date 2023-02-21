package org.firstinspires.ftc.teamcode.EKopmodes.archived;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="LeftAutonCycle", group="auto")
public class LeftAutonCycle extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        Robot wucru = new Robot(hardwareMap, this);

        wucru.initOpenCV();
        waitForStart();
        wucru.resetEncoders();

        telemetry.update();
        int sleeveColor = DetectorPipeline.sleeveColor;

        wucru.moveClaw();
        wucru.moveClaw();

        //code
        wucru.straight(1,30,0.2);
        wucru.strafe(1, 14,0.2);
        wucru.moveSlide(0.5, 5);
        wucru.straight(1,10,0.2);
        wucru.moveClaw();
        wucru.straight(-1,10,0.2);
        wucru.moveClaw();
        wucru.moveSlide(-0.5, 5);

        //cycle
        wucru.strafe(-1,14,0.2);
        wucru.straight(1,27,0.2);
        wucru.turnTo(-Math.PI/2,0.2);//check

        for(int i = 0; i <2;i++) {
            wucru.moveSlide(0.3, 2);//check
            wucru.moveClaw();
            wucru.straight(1, 30, 0.2);
            wucru.moveClaw();
            wucru.straight(-1, 50, 0.2);
            wucru.turnTo(Math.PI / 2, 0.2);//check
            wucru.moveSlide(0.3, 6);
            wucru.straight(1, 10, 0.2);
            wucru.moveClaw();
            wucru.straight(-1, 10, 0.2);
            wucru.moveClaw();
            wucru.moveSlide(-0.2, 5);
            wucru.turnTo(-Math.PI / 2, 0.2);//check
            wucru.straight(1, 20, 0.2);
        }


        //parking code
        if (sleeveColor == 1){

            wucru.straight(1,30,0.2);
            /*
            og code
            wucru.strafe(-1,27,0.5);
            wucru.resetEncoders();
            wucru.straight(1,30,0.5);
             */
        }
        if(sleeveColor == 2){
            wucru.straight(1,1,0.1);//does nothing
            /*
            wucru.straight(1, 28, 0.8);
            */
        }

        if (sleeveColor == 3){
            wucru.straight(-1,20,0.2);
            /*
            wucru.strafe(1,27,0.8);
            wucru.resetEncoders();
            wucru.straight(1, 30, 0.8);
             */
        }
    }
}
