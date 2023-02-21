package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.camera.LocalizationPipeline;

@TeleOp (name="Test Camera Localization", group="amongus")
public class TestCameraLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Robot wucru = new Robot(hardwareMap, this);
        wucru.initOpenCV();
        LocalizationPipeline test = new LocalizationPipeline(this);
        wucru.webcam.setPipeline(test);

        double distance;
        double angle;
        waitForStart();
        while(opModeIsActive()) {
            distance = wucru.distance.getDistance(DistanceUnit.INCH);
            angle = test.pixelsOffCenter / 394;
            telemetry.addData("Pixels Off From Center", test.pixelsOffCenter);
            telemetry.addData("Distance", distance);
            telemetry.addData("Angle", angle);
            telemetry.addData("Position", ("(" + distance + ", " + (distance * Math.tan(angle)) + ")"));
            telemetry.update();
        }
    }
}
