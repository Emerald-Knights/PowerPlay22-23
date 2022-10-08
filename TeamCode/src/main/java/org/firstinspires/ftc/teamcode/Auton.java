package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Auton", group="auto")
public class Auton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        robot bot = new robot();
        bot.initOpenCV(hardwareMap);
        waitForStart();
    }
}
