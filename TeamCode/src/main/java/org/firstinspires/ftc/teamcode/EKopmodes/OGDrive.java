package org.firstinspires.ftc.teamcode.EKopmodes;

import com.acmerobotics.dashboard.message.redux.StartOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp (name = "DriveSLOWMODE")
public class OGDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap, this);
        waitForStart();

        double lx, rx, ly; // intialize variables for the gamepad
        boolean clawClosed = false;
        boolean neckUp = false;
        double slowmode = 0.5;
        boolean lateA = false;
        boolean lateB = false;
        boolean lateX = false;
        boolean turned = false;

        double kp = 1;
        double ki = 0;
        double kd = 0.1;

        double targetAngle = 0;
        double cycleTime = 0;

        double error = 0;
        double prevError = 0;
        double iGain = 0;
        double pGain = 1;
        double dGain = 0;
        bot.timer.reset();
//        bot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            //drive
            // set the gamepad variables
            lx = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x;
            ly = -gamepad1.left_stick_y;
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


            //calculate cycle time
            cycleTime = bot.timer.milliseconds();
            bot.timer.reset();

            if (magnitude > 0.03) {
                if(Math.abs(rx) <= 0.03) {
                    //PID algo if it's not turning
                    if(turned){
                        //if it turned, recalibrate the target angle
                        targetAngle = bot.imu.getAngularOrientation().firstAngle;
                        turned = false;
                    }
                    error = targetAngle - bot.imu.getAngularOrientation().firstAngle;
                    iGain += ki * error * cycleTime;
                    dGain = kd * (error - prevError)/cycleTime;
                    pGain = kp * error;
                    prevError = error;

                    bot.leftFront.setPower((lf * ratio * 0.8 * slowmode) - (pGain + dGain + iGain));
                    bot.leftBack.setPower((lb * ratio * 0.8 * slowmode) - (pGain + dGain + iGain));
                    bot.rightFront.setPower((rf * ratio * 0.8 * slowmode) + (pGain + dGain + iGain));
                    bot.rightBack.setPower((rb * ratio * 0.8 * slowmode) + (pGain + dGain + iGain));
                }
                else{
                    //regular algo if it's turning
                    turned = true;
                    prevError = 0;

                    bot.leftFront.setPower((lf * ratio * 0.8 * slowmode));
                    bot.leftBack.setPower((lb * ratio * 0.8 * slowmode));
                    bot.rightFront.setPower((rf * ratio * 0.8 * slowmode));
                    bot.rightBack.setPower((rb * ratio * 0.8 * slowmode));
                }
            }
            else{
                bot.leftFront.setPower(0);
                bot.leftBack.setPower(0);
                bot.rightFront.setPower(0);
                bot.rightBack.setPower(0);
            }
            if(gamepad1.a && !lateA) {
                if (clawClosed) {
                    //open position
                    bot.rightClaw.setPosition(0.4);
                    bot.leftClaw.setPosition(0.2);
                    clawClosed=false;

                } else {
                    //closed position
                    bot.rightClaw.setPosition(0.1);
                    bot.leftClaw.setPosition(0.5);
                    clawClosed=true;

                }
            }

            if(gamepad1.b && !lateB){
                if (neckUp) {
                    //open position
                    bot.arm.setPosition(1);
                    neckUp=false;
                }
                else {
                    //closed position
                    bot.arm.setPosition(0.75);
                    neckUp=true;
                }
            }

            if(gamepad2.x && !lateX){
                if(slowmode == 0.5){
                    slowmode = 1;
                }
                else{
                    slowmode = 0.5;
                }
            }

            lateA = gamepad1.a;
            lateB = gamepad1.b;
            lateX = gamepad2.x;


        }
    }
}