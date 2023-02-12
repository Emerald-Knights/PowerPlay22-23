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
        while (opModeIsActive()) {
            //drive
            // set the gamepad variables
            lx = gamepad1.left_stick_x * 0.65;
            rx = gamepad1.right_stick_x * 0.5;
            ly = -gamepad1.left_stick_y * 0.65;
            // arithmetic to get motor values - not scaled
            double lf = ly + rx + lx;
            double lb = ly + rx - lx;
            double rf = ly - rx - lx;
            double rb = ly - rx + lx;
            // scale the motor values
            double ratio;
            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            if (max == 0) {
                ratio = 0;
            } else {
                ratio = .3 * magnitude / max;
            }
            // sets the motor power
            if (magnitude > 0.18) {
                bot.leftFront.setPower(lf * ratio*0.8);
                bot.leftBack.setPower(lb * ratio*0.8);
                bot.rightFront.setPower(rf * ratio*0.8);
                bot.rightBack.setPower(rb * ratio*0.8);
            }
            else{
                bot.leftFront.setPower(0);
                bot.leftBack.setPower(0);
                bot.rightFront.setPower(0);
                bot.rightBack.setPower(0);
            }
            //telemetry.addData("lb encoder:", bot.leftBack.getCurrentPosition());
            telemetry.addData("y val:", gamepad1.left_stick_y);
            telemetry.addData("lb:",lb);
            telemetry.addData("lf",lf);
            telemetry.addData("rf",rf);
            telemetry.addData("rb",rb);
            telemetry.addData("Distance(m): ", bot.distance.getDistance(DistanceUnit.METER));
            telemetry.addData("Distance(in): ", bot.distance.getDistance(DistanceUnit.INCH));

            telemetry.update();
            // slides
            if (gamepad2.right_trigger>0){
                bot.setSlidePower(gamepad2.right_trigger*0.4);
            }
            if(gamepad2.left_trigger>0){
                bot.setSlidePower(-gamepad2.left_trigger*0.4);
            }
            if(gamepad2.left_trigger==0 && gamepad2.right_trigger==0){
                bot.setSlidePower(0);
            }
            //servo
            if (gamepad2.a && !lateA){
                bot.moveClaw();
            }

            lateA = gamepad2.a;


        }
    }
}