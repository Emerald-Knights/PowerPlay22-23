package org.firstinspires.ftc.teamcode.EKopmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.camera.LocalizationPipeline;

@Autonomous(name="LeftAutonEncoderTicks", group="auto")
public class LeftAutonEncoderTicks extends LinearOpMode {

    Robot wucru;

    @Override
    public void runOpMode() throws InterruptedException {
        wucru = new Robot(hardwareMap, this);
        wucru.initOpenCV();
        double distance;
        double angle;
        double x;

        waitForStart();



        //start sequence

        //pick up preloaded cone
        wucru.moveNeck();
        wucru.moveClaw();
        sleep(1000);

        //start lifting slide
        wucru.slide.setTargetPosition(117 * 38);
        wucru.slide.setPower(0.7);
        sleep(1000);


        //init opencv
        LocalizationPipeline camera = new LocalizationPipeline(this);
        wucru.webcam.setPipeline(camera);

        telemetry.addData("sleevecolor", DetectorPipeline.sleeveColor);
        telemetry.update();


        //drive to first junction
        wucru.straightOneOdo(1,0.4,51, 0, 0.08, true);
        sleep(500);
        wucru.turnTo(Math.PI/4, 0.4, -1);


        //deposit first cone
        wucru.straightOneOdo(1, 0.3, 6.7, Math.PI/4, 0, true);
        sleep(1000);

        //lower slides while over junction
        wucru.slide.setTargetPosition(117 * 27);
        wucru.slide.setPower(-0.5);
        sleep(1000);

        //let go of cone
        wucru.moveClaw();
        sleep(500);

        //back up & start lowering slides
        wucru.slide.setTargetPosition(117 * 4);
        wucru.slide.setPower(-0.3);
        wucru.straightOneOdo(-1, 0.25, 8, Math.PI/4,0, true);
        sleep(1000);

        //turn toward cone stack
        wucru.turnTo(-Math.PI/2 +.08, 0.5);
        sleep(500);

        //turnto again to adjust for error
        wucru.turnTo(-Math.PI/2, 0.25);

        //camera localization sequence

//        distance = wucru.distance.getDistance(DistanceUnit.INCH);
//        angle = camera.pixelsOffCenter / 394;
//        x = (distance - 4) * Math.tan(angle)-2;
//
//        telemetry.addData("x", x);
//        telemetry.addData("angle", angle);
//        telemetry.addData("distance", distance);
        telemetry.addData("color", DetectorPipeline.sleeveColor);
        telemetry.update();
//        sleep(1000);


        //move to cone stack
        sleep(1000);
        wucru.straightOneOdo(1, 0.4, 21, wucru.imu.getAngularOrientation().firstAngle, 0, true);
        sleep(1000);
        wucru.moveClaw();
        sleep(500);
        wucru.slide.setTargetPosition(117 * 36);
        wucru.slide.setPower(0.5);
        sleep(800);

        //go to junction for 2nd cycle
        wucru.straightOneOdo(-1, 0.4, 22, wucru.imu.getAngularOrientation().firstAngle, 0, true);

        //rotate to junction second time
        wucru.moveNeck();
        wucru.turnTo(-3*Math.PI/4 + 0.05, 0.3);
        wucru.straightOneOdo(-1, 0.3, 5, -3*Math.PI/4, 0.03, true);
        sleep(1500);

        //drop cone second time
        wucru.slide.setTargetPosition(117 * 28);
        wucru.slide.setPower(-0.5);
        sleep(1000);
        wucru.moveClaw();

        //reset slide
        wucru.slide.setTargetPosition(117 * 4);
        wucru.slide.setPower(-0.5);

        //back up from junction
        wucru.straightOneOdo(1, 0.3, 7, -3*Math.PI/4, 0.03, true);
        wucru.turnTo(-Math.PI/2, 0.5);
//        park

        //temporary park testing
//        DetectorPipeline.sleeveColor = 3;

//        if(DetectorPipeline.sleeveColor == 3){
            wucru.straightOneOdo(-1, 0.5, 5, -Math.PI/2, 0.03, true);
//        }
//        else if (DetectorPipeline.sleeveColor == 1){
//            wucru.straightOneOdo(1, 0.5, 11, -Math.PI/2, 0.03, true);
//        }

        telemetry.update();

    }
}