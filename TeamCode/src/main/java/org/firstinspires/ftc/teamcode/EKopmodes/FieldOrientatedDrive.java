package org.firstinspires.ftc.teamcode.EKopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp (name = "AbsolutelySupremeSoftware ", group = "amongus")
public class FieldOrientatedDrive extends LinearOpMode {

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

                    double maxInput = Math.max(Math.max(Math.abs(lx), Math.abs(ly)), Math.abs(rx));
                    double direction = Math.atan2(-ly, lx); //set direction
                    double lf = Math.sin(currentAngle + Math.PI*3/4 + direction) * maxInput;
                    double rf = Math.sin(currentAngle + Math.PI*5/4 + direction) * maxInput;
                    double turnPower = -rx; //turn power can be changed to a magnitude and direction
                    double denom = turnPower + Math.max(Math.abs(rf), Math.abs(lf));
                    double translateRatio= Math.pow(Math.max(rf, lf), 2)/denom;
                    double rotateRatio = Math.pow(turnPower, 2)/denom;

                    telemetry.addData("translateRatio", translateRatio);
                    telemetry.addData("rotateRatio", rotateRatio);
                    telemetry.addData("turnPower", turnPower);
                    telemetry.addData("lf", lf);
                    telemetry.addData("rf", rf);

                    wucru.leftFront.setPower(0.8 * ((translateRatio * lf) + (turnPower * rotateRatio)));
                    wucru.leftBack.setPower(0.8 * ((translateRatio * rf) + (turnPower * rotateRatio)));
                    wucru.rightFront.setPower(0.8 * ((translateRatio * rf) - (turnPower * rotateRatio)));
                    wucru.rightBack.setPower(0.8 * ((translateRatio * lf) - (turnPower * rotateRatio)));

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
                    PIDisActive = !wucru.slideUpdate();
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
