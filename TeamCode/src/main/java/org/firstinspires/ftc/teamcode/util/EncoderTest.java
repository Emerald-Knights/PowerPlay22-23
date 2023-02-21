package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Encoder Test ", group = "amongus")
public class EncoderTest extends LinearOpMode{
        @Override
        public void runOpMode() throws InterruptedException {
                Robot aicul = new Robot(hardwareMap, this);
                waitForStart();
                while(opModeIsActive()) {
                        if (gamepad1.left_trigger > 0)
                                aicul.leftFront.setPower(0.1);
                        else
                                aicul.leftFront.setPower(0);
                        telemetry.addData("lf", aicul.leftFront.getCurrentPosition());

                        if (gamepad1.right_trigger > 0)
                                aicul.rightFront.setPower(0.1);
                        else
                                aicul.rightFront.setPower(0);
                        telemetry.addData("rf", aicul.rightFront.getCurrentPosition());

                        telemetry.addData("port1", aicul.rightFront.getCurrentPosition());
                        telemetry.update();
                }
        }
}
