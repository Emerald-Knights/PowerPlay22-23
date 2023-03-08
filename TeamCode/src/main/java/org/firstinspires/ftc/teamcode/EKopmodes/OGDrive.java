package org.firstinspires.ftc.teamcode.EKopmodes;

import com.acmerobotics.dashboard.message.redux.StartOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp (name = "DazzlingIdealCapableKnowledgeableSoftware")
public class OGDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, this);
        waitForStart();

        double lx, rx, ly; // intialize variables for the gamepad
        boolean lateA = false;
        boolean lateB = false;
        boolean lateDdown = false;
        boolean lateDleft = false;
        boolean lateDright = false;
        boolean lateDup = false;
        boolean lateX = false;
        bot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            //drive
            // set the gamepad variables
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            ly = gamepad1.left_stick_y;
            // arithmetic to get motor values - not scaled
            double lf = ly - rx - lx;
            double lb = ly - rx + lx;
            double rf = ly + rx + lx;
            double rb = ly + rx - lx;
            // scale the motor values
            double ratio;
            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            if (max == 0) {
                ratio = 0;
            } else {
                ratio = magnitude / max;
            }
            // sets the motor power
            if (magnitude > 0.05) {
                bot.leftFront.setPower(lf * ratio*0.6);
                bot.leftBack.setPower(lb * ratio*0.6);
                bot.rightFront.setPower(rf * ratio*0.6);
                bot.rightBack.setPower(rb * ratio*0.6);
            }
            else{
                bot.leftFront.setPower(0);
                bot.leftBack.setPower(0);
                bot.rightFront.setPower(0);
                bot.rightBack.setPower(0);
            }
            //telemetry.addData("lb encoder:", bot.leftBack.getCurrentPosition());
            telemetry.addData("RF: ", bot.rightFront.getPower());
            telemetry.addData("RB: ", bot.rightBack.getPower());
            telemetry.addData("LF: ", bot.leftFront.getPower());
            telemetry.addData("LB: ", bot.leftBack.getPower());


            telemetry.addData("slideEncoder: ", bot.slide.getCurrentPosition());
            //bottom: 0: top around 4181  mid:2985 bottom:1679
            telemetry.addData("y val:", gamepad1.left_stick_y);
            telemetry.addData("lb:",lb);
            telemetry.addData("lf",lf);
            telemetry.addData("rf",rf);
            telemetry.addData("rb",rb);
            telemetry.addData("left:", bot.leftOdo.getCurrentPosition());
            telemetry.addData("right:", bot.rightOdo.getCurrentPosition());
            telemetry.addData("center:", bot.centerOdo.getCurrentPosition());
            telemetry.addData("slidePower", bot.slide.getPower());

            telemetry.update();
            bot.setSlidePower((gamepad2.right_trigger - gamepad2.left_trigger) * 0.8);
            if (gamepad2.a && !lateA){
                bot.moveClaw();
            }
            if (gamepad2.b && !lateB) {
                bot.moveNeck();
            }
            lateB = gamepad2.b;
            lateA = gamepad2.a;
            lateX = gamepad2.x;
            lateDdown = gamepad2.dpad_down;
            lateDleft = gamepad2.dpad_left;
            lateDright = gamepad2.dpad_right;
            lateDup = gamepad2.dpad_up;


        }
    }
}