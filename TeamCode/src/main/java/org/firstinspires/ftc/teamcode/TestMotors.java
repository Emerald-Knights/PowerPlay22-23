package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class TestMotors {
    @TeleOp(name = "Test Motors", group = "amongus")
    public class Drive extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            Robot wucru = new Robot(hardwareMap, this);
            waitForStart();

            while (opModeIsActive()) {
                
            }
        }
    }
}
