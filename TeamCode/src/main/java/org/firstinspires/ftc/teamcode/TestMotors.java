package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test Motors", group = "amongus")
public class TestMotors extends LinearOpMode{
        @Override
        public void runOpMode() throws InterruptedException {
            Robot wucru = new Robot(hardwareMap, this);
            waitForStart();
            boolean lateA = false;
            boolean lateB = false;
            while (opModeIsActive()) {
//                if(gamepad1.a && !lateA) {
//                    wucru.moveSlide(0.4, 2);
//                } else if (gamepad1.b && !lateB){
//                    wucru.moveSlide(-0.4, 2);
//                }
                if(gamepad1.a) {
                    wucru.rightFront.setPower(0.3);
                } else {
                    wucru.rightFront.setPower(0);
                }
                lateA = gamepad1.a;
                lateB = gamepad1.b;
                telemetry.addData("pow", wucru.rightFront.getPower());
                telemetry.update();
            }
        }
}
