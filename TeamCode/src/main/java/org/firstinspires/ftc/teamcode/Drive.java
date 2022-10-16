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

            double direction = Math.atan2(-ly, lx);
            double lf = Math.sin(currentAngle + Math.PI*3/4 + direction);
            double rf = Math.sin(currentAngle + Math.PI*5/4 + direction);

            double ratio;
            double max = Math.max(Math.abs(rf), Math.abs(lf));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly));
            ratio = (max == 0) ? 0 : magnitude / max * 0.8;

            wucru.leftFront.setPower((lf+rx)*ratio);
            wucru.leftBack.setPower((rf+rx)*ratio);
            wucru.rightFront.setPower((rf-rx)*ratio);
            wucru.rightBack.setPower((lf-rx)*ratio);
        }
    }
}
