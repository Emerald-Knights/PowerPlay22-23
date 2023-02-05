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
        PID,
        RNP
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Robot wucru = new Robot(hardwareMap, this);
        RobotState robotState = RobotState.DRIVE;
        waitForStart();

        boolean clawLate = false;
        boolean xLate = false;
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

                    double translateRatio = Math.pow(Math.max(rf, lf), 2)/(turnPower+Math.max(rf,lf));
                    double rotateRatio = Math.pow(turnPower, 2)/(turnPower+Math.max(rf, lf));

                    wucru.leftFront.setPower(0.8 * ((translateRatio * lf) + (turnPower * rotateRatio)));
                    wucru.leftBack.setPower(0.8 * ((translateRatio * rf) + (turnPower * rotateRatio)));
                    wucru.rightFront.setPower(0.8 * ((translateRatio * rf) - (turnPower * rotateRatio)));
                    wucru.rightBack.setPower(0.8 * ((translateRatio * lf) - (turnPower * rotateRatio)));

                    if(gamepad2.b && !clawLate){
                        robotState = RobotState.CLAW;
                    } else if(gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                        robotState = RobotState.SLIDE;
                    } else if(gamepad2.x && !xLate){
                        robotState = RobotState.RNP;
                    }
                    break;
                case SLIDE:
                    wucru.setSlidePower(gamepad2.right_trigger - gamepad2.left_trigger);
                    robotState = RobotState.DRIVE;
                    break;
                case CLAW:
                    wucru.moveClaw();
                    robotState = RobotState.DRIVE;
                    break;
                case RNP:
                    wucru.moveRack();
                    robotState = RobotState.DRIVE;
                    break;

            }

            //set late
            clawLate = gamepad2.b;
            xLate = gamepad2.x;
            telemetry.addData("servo", wucru.leftClaw.getPosition());
            telemetry.addData("rservo", wucru.rightClaw.getPosition());
            telemetry.update();
        }
    }
}
