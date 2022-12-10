package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name="Distance Sensor", group="ur mom")
public class Distance extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        Robot keshav = new Robot(hardwareMap, this);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Distance", keshav.distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
