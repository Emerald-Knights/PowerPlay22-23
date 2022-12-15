package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="TicksToInches", group="sus")
public class TicksToInches extends LinearOpMode {

    int ticks;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot wucru = new Robot(hardwareMap, this);
        waitForStart();
        int ticks = 1000;
        wucru.resetEncoders();
//        wucru.strafe(-1, ticks, 0.8);

        telemetry.addData("Ticks:", ticks);
        telemetry.update();
    }
}
