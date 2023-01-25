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
            Robot wucru = new Robot(hardwareMap, this, telemetry);
            waitForStart();
            boolean lateA = false;
            boolean lateB = false;
            boolean moveSlides = false;
            wucru.slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wucru.slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while (opModeIsActive()) {
                wucru.targetSlidePosition = 1;
                if(gamepad1.a) {
                    moveSlides = true;
                } else if(gamepad1.b) {
                    moveSlides = false;
                }
                if(moveSlides) {
                    wucru.slideUpdate();
                }
                telemetry.addData("slide status", moveSlides);
                telemetry.addData("slide1", wucru.slide1.getCurrentPosition());
                telemetry.addData("slide2", wucru.slide2.getCurrentPosition());
                lateA = gamepad1.a;
                telemetry.update();
            }
        }
}
