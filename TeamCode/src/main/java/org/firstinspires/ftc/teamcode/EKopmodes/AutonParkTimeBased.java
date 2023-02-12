package org.firstinspires.ftc.teamcode.EKopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="AutonParkWithTime", group="auto")
public class AutonParkTimeBased extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        Robot wucru = new Robot(hardwareMap, this);

        wucru.initOpenCV();
        waitForStart();
        wucru.resetEncoders();

        int sleeveColor = DetectorPipeline.sleeveColor;

        sleep(1000);

        telemetry.addData("detected: ", sleeveColor);
        sleep(1000);
        if (sleeveColor == 1){
            telemetry.addLine("in 1 loop");
            wucru.strafeWtime(-1,0.2,0.8);
            sleep(1000);
            telemetry.addData("heading: ", wucru.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            wucru.turnTo(0, 0.08);
            sleep(1000);
            wucru.straightWtime(1,0.1,1);
        }
        else if(sleeveColor == 2){
            telemetry.addLine("in 2 loop");
            wucru.straightWtime(1, 0.1, 1.3);
        }
        else if (sleeveColor == 3){
            telemetry.addLine("in 3 loop");
            wucru.strafeWtime(1,0.2,0.8);
            sleep(1000);
            telemetry.addData("heading: ", wucru.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            wucru.turnTo(0, 0.08);
            sleep(1000);
            wucru.straightWtime(1,0.1,1.1);
        }



    }
}
