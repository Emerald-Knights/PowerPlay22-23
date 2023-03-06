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
            //Robot wucru = new Robot(hardwareMap, this);
            DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            waitForStart();
            while (opModeIsActive()) {
                slide.setPower((gamepad1.right_trigger - gamepad1.left_trigger) * 0.8);
                telemetry.addData("slide status", slide.getPower());
                telemetry.update();
            }
        }
}
