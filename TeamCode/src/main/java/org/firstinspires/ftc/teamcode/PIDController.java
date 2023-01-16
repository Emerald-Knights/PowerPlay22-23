package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastFilterEstimate = 0;
    private double a = 1;
    private ElapsedTime time = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDController(double kP, double kI, double kD, double a) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.a = a;
    }

    public double update(double target, double state) {
        double error = target - state;
        double derivative = (lowPassFilter(error - lastError)) / time.seconds();
        integralSum += error * time.seconds();
        double out = (kP * error) + (kI * integralSum) + (kD * derivative);
        lastError = error;
        time.reset();
        return out;
    }

    public double logUpdate(double target, double state, Telemetry telemetry, String name) {
        double out = update(target, state);
        telemetry.addData(name, state);
        return out;
    }

    public double lowPassFilter(double deltaError) {
        double currFilterEstimate = (a * lastFilterEstimate) + (1-a) * deltaError;
        lastFilterEstimate = currFilterEstimate;
        return currFilterEstimate;
    }
}
