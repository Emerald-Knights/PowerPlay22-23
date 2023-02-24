package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Servo Programming", group="auto")
public class ServoProgramming extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        Robot wucru = new Robot(hardwareMap, this);
        waitForStart();
        boolean lateA = false;
        boolean lateB = false;
        double currentPos = 0.5;
        while(opModeIsActive()) {
            if(gamepad1.a && !lateA) {
                currentPos -= 0.0002;
            }
            else if(gamepad1.b && !lateB) {
                currentPos += 0.0002;
            }
            wucru.neck.setPosition(currentPos);
            telemetry.addData("Current Position", currentPos);
            telemetry.addData("Servo: ", wucru.neck.getPosition());
            telemetry.update();
        }
    }
}