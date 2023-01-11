package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TicksToInches;

public class OdometryLocalizer implements Localizer {

    Pose2d poseEstimate;
    DcMotor front, left, right;

    final static double L = 0; // distance between left and right
    final static double B = 0; // distance between midpoint and front
    final static double R = 0; // radius of wheels
    final static double N = 8192; //encoder ticks
    final static double INCH_PER_TICK = 2.0 * Math.PI * R / N; // you don't need R and N if you find this value through testing
    private int oldRightEncoder;
    private int oldLeftEncoder;
    private int oldFrontEncoder;

    public OdometryLocalizer(HardwareMap hardwareMap) {
        front = hardwareMap.get(DcMotor.class, "odoFront");
        left = hardwareMap.get(DcMotor.class, "odoLeft");
        right = hardwareMap.get(DcMotor.class, "odoRight");
    }

    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        //add negatives to reverse values
        int currRightEncoder = right.getCurrentPosition();
        int currLeftEncoder = left.getCurrentPosition();
        int currFrontEncoder = front.getCurrentPosition();

        int deltaRight = currRightEncoder - oldRightEncoder;
        int deltaLeft = currLeftEncoder - oldLeftEncoder;
        int deltaFront = currFrontEncoder - oldFrontEncoder;

        double dTheta = INCH_PER_TICK * (deltaRight - deltaLeft) / L;
        double dx = INCH_PER_TICK * (deltaRight + deltaLeft) / 2.0;
        double dy = INCH_PER_TICK * (deltaFront - (deltaLeft - deltaRight) * B / L);

        poseEstimate = new Pose2d(dx + poseEstimate.getX(), dy + poseEstimate.getY(), dTheta + poseEstimate.getHeading());

        oldRightEncoder = currRightEncoder;
        oldLeftEncoder = currLeftEncoder;
        oldFrontEncoder = currFrontEncoder;
    }
}
