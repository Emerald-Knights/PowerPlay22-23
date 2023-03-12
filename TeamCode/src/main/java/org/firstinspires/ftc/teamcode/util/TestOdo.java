package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.rrutil.Encoder;

@Config
@TeleOp(name = "Test Odo", group = "amongus")
public class TestOdo extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
//        Encoder leftOdo = new Encoder(hardwareMap.get(DcMotorEx.class, "leftOdo"));
        Encoder centerOdo = new Encoder(hardwareMap.get(DcMotorEx.class,"centerOdo"));
        Encoder rightOdo = new Encoder(hardwareMap.get(DcMotorEx.class,"rightOdo"));
        rightOdo.motor.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
//            telemetry.addData("leftOdo", leftOdo.motor.getCurrentPosition());
            telemetry.addData("centerOdo",centerOdo.motor.getCurrentPosition());
            telemetry.addData("rightOdo", rightOdo.motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
