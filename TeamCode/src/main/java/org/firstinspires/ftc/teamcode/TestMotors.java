package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Motors", group = "amongus")
public class TestMotors extends LinearOpMode{
        @Override
        public void runOpMode() throws InterruptedException {
            Robot wucru = new Robot(hardwareMap, this);
            waitForStart();
            boolean lateA = false;
            while (opModeIsActive()) {
                if(gamepad1.a && !lateA) {
                    wucru.moveSlide(0.4, 2);
                }
                lateA = gamepad1.a;
            }
        }
}
