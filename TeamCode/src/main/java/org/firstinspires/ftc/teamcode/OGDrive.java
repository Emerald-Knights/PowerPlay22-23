package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp (name = "DazzlingIdealCapableKnowledgeableSoftware")
public class OGDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, this);
        waitForStart();

        double lx, rx, ly; // intialize variables for the gamepad
        boolean lateX = false;
        boolean lateB = false;
        boolean armUp = false;
        boolean wristClose = false;
        while (opModeIsActive()) {
            //drive

            // set the gamepad variables
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            ly = -gamepad1.left_stick_y;
            // arithmetic to get motor values - not scaled
            double lf = -ly - rx - lx;
            double lb = -ly - rx + lx;
            double rf = -ly + rx + lx;
            double rb = -ly + rx - lx;
            // scale the motor values
            double ratio;
            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            if (max == 0) {
                ratio = 0;
            } else {
                ratio = .8 * magnitude / max;
            }
            // sets the motor power
//            bot.leftFront.setPower(lf * ratio);
//            bot.leftBack.setPower(lb * ratio);
//            bot.rightFront.setPower(rf * ratio);
//            bot.rightBack.setPower(rb * ratio);

            //arm/wrist
            if(gamepad2.x && !lateX) {
                if(wristClose) {
                    bot.moveWrist(false);
                    wristClose = !wristClose;
                } else {
                    bot.moveWrist(true);
                    wristClose = !wristClose;
                }
            }
//            if(gamepad2.b && !lateB) {
//                if(armUp) {
//                    bot.moveArm(false);
//                    armUp = !armUp;
//                } else {
//                    bot.moveArm(true);
//                    armUp = !armUp;
//                }
//            }

            lateX = gamepad2.x;
            lateB = gamepad2.b;
        }
    }
}