package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.android.dx.cf.attrib.AttEnclosingMethod;

@TeleOp (name = "AbsolutelySupremeSoftware ", group = "amongus")
public class Drive extends LinearOpMode {

    boolean lateA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot wucru = new Robot(hardwareMap, this);
        waitForStart();

        while (opModeIsActive()) {

            //drive
            double lx=gamepad1.left_stick_x;
            double ly=-gamepad1.left_stick_y;
            double rx=-gamepad1.right_stick_x;
            double currentAngle = wucru.imu.getAngularOrientation().firstAngle;

            double direction = Math.atan2(-ly, lx); //set direction
            double lf = Math.sin(currentAngle + Math.PI*3/4 + direction);
            double rf = Math.sin(currentAngle + Math.PI*5/4 + direction);
            double turnPower = rx; //turn power can be changed to a magnitude and direction

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

            //claw
            if(!lateA && gamepad2.a) {
                wucru.moveClaw();
            }

            //set late
            lateA = gamepad2.a;

            telemetry.update();
        }
    }
}
