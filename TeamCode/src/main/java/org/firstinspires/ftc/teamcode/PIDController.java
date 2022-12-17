package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime time = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double target, double state) {
        double error = target - state;
        double derivative = (error - lastError) / time.seconds();
        integralSum += error * time.seconds();
        double out = (kP * error) + (kI * integralSum) + (kD * derivative);
        lastError = error;
        time.reset();
        return out;
    }

    public double logUpdate(double target, double state, FtcDashboard dashboard, String name) {
        double error = target - state;
        double derivative = (error - lastError) / time.seconds();
        integralSum += error * time.seconds();
        double out = (kP * error) + (kI * integralSum) + (kD * derivative);
        lastError = error;
        time.reset();

        Telemetry telemetry;
        telemetry = dashboard.getTelemetry();
        telemetry.addData(name, state);

        return out;
    }
}
