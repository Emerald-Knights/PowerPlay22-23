package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "AbsolutelySupremeSoftware ", group = "amongus")
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        robot wucru = new robot(hardwareMap, this);
        waitForStart();

        while (opModeIsActive()) {

            double lx=gamepad1.left_stick_x;
            double ly=-gamepad1.left_stick_y;
            double rx=gamepad1.right_stick_x;
            double currentAngle = wucru.imu.getAngularOrientation().firstAngle;

            double direction = Math.atan2(-ly, lx); //set direction
            // direction = tan(ly/lx)
            double lf = Math.sin(currentAngle + Math.PI*3/4 + direction);
            double rf = Math.sin(currentAngle + Math.PI*5/4 + direction);
            double turnPower = rx; //turn power can be changed to a magnitude and direction

            double ratio;
            double max = Math.max(Math.abs(rf), Math.abs(lf));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            if (max == 0) {
                ratio = 0;
            }
            else {
                ratio = magnitude / max ;
            }
            if(lx < 0.02 && lx > -0.02 && ly < 0.02 && ly > -0.02){
                wucru.leftFront.setPower(0.8 * turnPower);
                wucru.leftBack.setPower(0.8 * turnPower);
                wucru.rightFront.setPower(0.8 * -turnPower);
                wucru.rightBack.setPower(0.8 * -turnPower);
            }
            else {
                wucru.leftFront.setPower(lf * ratio + turnPower);
                wucru.leftBack.setPower(rf * ratio + turnPower);
                wucru.rightFront.setPower(rf * ratio - turnPower);
                wucru.rightBack.setPower(lf * ratio - turnPower);
            }
        }
    }
}
