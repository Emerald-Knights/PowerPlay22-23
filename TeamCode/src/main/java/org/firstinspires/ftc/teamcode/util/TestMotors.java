package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Test Motors", group = "amongus")
public class TestMotors extends LinearOpMode{
        @Override
        public void runOpMode() throws InterruptedException {
            Robot wucru = new Robot(hardwareMap, this);
            waitForStart();
            while (opModeIsActive()) {
//                if(gamepad1.a) {
//                    moveSlides = true;
//                } else if(gamepad1.b) {
//                    moveSlides = false;
//                }
//                telemetry.addData("slide status", moveSlides);
//                lateA = gamepad1.a;
                telemetry.update();
            }
        }
}
