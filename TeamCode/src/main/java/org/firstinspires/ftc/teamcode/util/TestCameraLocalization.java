package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.camera.LocalizationPipeline;

@TeleOp (name="Test Camera Localization", group="amongus")
public class TestCameraLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Robot wucru = new Robot(hardwareMap, this);
        wucru.initOpenCV();
        wucru.webcam.setPipeline(new LocalizationPipeline());
        waitForStart();
    }
}
