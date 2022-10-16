package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "AbsolutelySupremeSoftware ", group = "amongus")
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot woofWoof = new Robot(hardwareMap, this);
        waitForStart();

        double lx, rx, ly;
        while (opModeIsActive()) {

            // set the gamepad variables
            lx = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            ly = -gamepad1.left_stick_y;


            telemetry.addData("heading", woofWoof.imu.getAngularOrientation().firstAngle);
            telemetry.addData("heading3", woofWoof.imu.getAngularOrientation().thirdAngle);
            double angle = woofWoof.imu.getAngularOrientation().thirdAngle;
            telemetry.update();

            // do spinny thing
            double direction = Math.PI; //set direction
            // direction = tan(y/x)
            double lf = Math.sin(angle + Math.PI * 3/4 + direction);
            double rf = Math.sin(angle + Math.PI * 5/4 + direction);
            double turnPower = 1; //turn power can be changed to a magnitude and direction
           woofWoof.leftFront.setPower(lf + turnPower);
           woofWoof.leftBack.setPower(rf + turnPower);
           woofWoof.rightFront.setPower(rf - turnPower);
           woofWoof.rightBack.setPower(lf - turnPower);
        }
    }
}
