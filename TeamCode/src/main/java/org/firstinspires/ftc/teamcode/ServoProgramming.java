package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Servo Programming", group="auto")
public class ServoProgramming extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        Robot wucru = new Robot(hardwareMap, this);
        waitForStart();
        boolean lateA = false;
        boolean lateB = false;
        double currentPos = 0.5;
        while(opModeIsActive()) {
            if(gamepad1.a && !lateA) {
                currentPos -= 0.0002;
            }
            else if(gamepad1.b && !lateB) {
                currentPos += 0.0002;
            }
<<<<<<< HEAD
            wucru.rightClaw.setPosition(currentPos);
=======
            wucru.leftClaw.setPosition(currentPos);
>>>>>>> dd56ea4efb00e8e2199f33a107d62c5a0872c673
            telemetry.addData("Current Position", currentPos);
            telemetry.update();
        }
    }
}
