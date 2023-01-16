package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.android.dx.cf.attrib.AttEnclosingMethod;

@TeleOp (name = "AbsolutelySupremeSoftware ", group = "amongus")
public class Drive extends LinearOpMode {

    boolean lateA = false;

    public enum RobotState{
        DRIVE,
        CLAW,
        SLIDE,
        PID
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot wucru = new Robot(hardwareMap, this);
        RobotState robotState = RobotState.DRIVE;
        waitForStart();

        boolean clawLate = false;
        boolean PIDisActive = false;
        while (opModeIsActive()) {

            switch (robotState){
                case DRIVE:
                    //drive
                    double lx=gamepad1.left_stick_x;
                    double ly=-gamepad1.left_stick_y;
                    double rx=-gamepad1.right_stick_x;
                    double currentAngle = wucru.imu.getAngularOrientation().firstAngle;

                    double direction = Math.atan2(-ly, lx); //set direction
                    double lf = Math.sin(currentAngle + Math.PI*3/4 + direction);
                    double rf = Math.sin(currentAngle + Math.PI*5/4 + direction);
                    double turnPower = -rx; //turn power can be changed to a magnitude and direction

                    double ratio;
                    //double max = Math.max(Math.abs(rf), Math.abs(lf)) + Math.abs(turnPower);
                    double max = Math.max(Math.abs(rf), Math.abs(lf)) + Math.abs(rx);
                    double magnitude = Math.sqrt((lf * lf) + (rf * rf) + (rx * rx));
                    if (max == 0) {
                        ratio = 0;
                    }
                    else {
                        ratio = 0.8 * magnitude / max ;
                    }
                    ratio = Math.max(Math.max(Math.abs(rf), Math.abs(lf)), Math.abs(rx)) * (0.8/max);

                    //deadzone
                    if(lx < 0.02 && lx > -0.02 && ly < 0.02 && ly > -0.02){
                        wucru.leftFront.setPower(0.8 * turnPower);
                        wucru.leftBack.setPower(0.8 * turnPower);
                        wucru.rightFront.setPower(0.8 * -turnPower);
                        wucru.rightBack.setPower(0.8 * -turnPower);
                    }
                    else {
                        wucru.leftFront.setPower(ratio*(lf + turnPower));
                        wucru.leftBack.setPower(ratio*(rf + turnPower));
                        wucru.rightFront.setPower(ratio*(rf - turnPower));
                        wucru.rightBack.setPower(ratio*(lf - turnPower));
                    }

                    if (gamepad2.dpad_down){
                        robotState = RobotState.SLIDE;
                        wucru.targetSlidePosition = 0;
                    }
                    else if(gamepad2.dpad_left){
                        robotState = RobotState.SLIDE;
                        wucru.targetSlidePosition = 1;
                    }
                    else if(gamepad2.dpad_up){
                        robotState = RobotState.SLIDE;
                        wucru.targetSlidePosition = 2;
                    }
                    else if(gamepad2.dpad_right){
                        robotState = RobotState.SLIDE;
                        wucru.targetSlidePosition = 3;
                    }
                    else if(PIDisActive){
                        robotState = RobotState.PID;
                    }
                    else if(gamepad2.b && !clawLate){
                        robotState = RobotState.CLAW;
                    }
                    break;
                case PID:
                    PIDisActive = !wucru.PIDupdate();
                    robotState = RobotState.DRIVE;
                    break;
                case SLIDE:
                    PIDisActive = true;
                    robotState = RobotState.PID;
                    break;
                case CLAW:
                    wucru.moveClaw();
                    robotState = RobotState.DRIVE;
                    break;

            }

            //set late
            clawLate = gamepad2.b;

            telemetry.addData("servo", wucru.leftClaw.getPosition());
            telemetry.addData("rservo", wucru.rightClaw.getPosition());
            telemetry.update();
        }
    }
}
