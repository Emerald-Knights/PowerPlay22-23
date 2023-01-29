package org.firstinspires.ftc.teamcode.EKopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp (name = "SLOWMODE")
public class Slowmode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, this);
        waitForStart();

        double lx, rx, ly; // intialize variables for the gamepad
        boolean lateX = false;
        boolean lateB = false;
        boolean slowModeLate = false;
        boolean armUp = false;
        boolean wristClose = false;
        boolean slowMode = false;
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
            if(slowMode){
                bot.leftFront.setPower(lf/2 * ratio);
                bot.leftBack.setPower(lb/2 * ratio);
                bot.rightFront.setPower(rf/2 * ratio);
                bot.rightBack.setPower(rb/2 * ratio);
            }
            else{
                bot.leftFront.setPower(lf * ratio);
                bot.leftBack.setPower(lb * ratio);
                bot.rightFront.setPower(rf * ratio);
                bot.rightBack.setPower(rb * ratio);
            }


            //arm/wrist

            lateX = gamepad2.x;
            lateB = gamepad2.b;
            slowModeLate= gamepad1.a;
        }
    }
}