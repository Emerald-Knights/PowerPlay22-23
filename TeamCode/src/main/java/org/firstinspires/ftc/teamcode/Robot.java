package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.camera.DetectorPipeline;
import org.firstinspires.ftc.teamcode.camera.LocalizationPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rrutil.Encoder;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot {
    public DcMotorEx leftBack, leftFront, rightBack, rightFront;
    public Servo leftClaw, rightClaw, arm;

    public DistanceSensor distance;
    public BNO055IMU imu;
    Orientation currentAngle;
    public ElapsedTime timer;

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    final static double TICKS_TO_INCH_FORWARD = 0.0265;
    final static double TICKS_TO_INCH_STRAFE = 0.01975;


    public OpenCvWebcam webcam;

    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        arm = hardwareMap.get(Servo.class, "arm");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
        telemetry = linearOpMode.telemetry;
        timer = new ElapsedTime();
    }


    public void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.setPipeline(new DetectorPipeline());
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        OpenCvWebcam finalWebcam = webcam;
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                finalWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }


        });
    }

    public void cameraLocalize() {
        double dist = distance.getDistance(DistanceUnit.INCH);
        double x = dist;
        double y = dist * LocalizationPipeline.RAD_PER_PIXEL;
//        setPoseEstimate(new Pose2d(x, y, imu.getAngularOrientation().firstAngle));
    }


    //auton methods
    public static double angleCompare(double angle1, double angle2) {
        double greaterAngle = Math.max(angle1, angle2);
        double smallerAngle = Math.min(angle1, angle2);
        if ((greaterAngle - smallerAngle) / (Math.PI * 2) > 1) {
            greaterAngle -= (int) ((greaterAngle - smallerAngle) / (Math.PI * 2)) * (Math.PI * 2);
        }
        return Math.min(greaterAngle - smallerAngle, Math.abs(greaterAngle - Math.PI * 2 - smallerAngle));
    }

    public static double factorial(int num) {
        int product = 1;
        for (int i = num; i >= 1; i--) {
            product *= i;
        }
        return product;
    }

    public static double sin(double angle) {
        double sum = 0;
        for (int i = 1; i < 17; i += 4) {
            sum += (Math.pow(angle, i) / factorial(i)) - (Math.pow(angle, i + 2) / factorial(i + 2));
        }
        return (sum);
    }

    public static double cos(double angle) {
        double sum = 0;
        for (int i = 0; i < 16; i += 4) {
            sum += (Math.pow(angle, i) / factorial(i)) - (Math.pow(angle, i + 2) / factorial(i + 2));
        }
        return (sum);
    }

    public int averageTicks() {
        return ((Math.abs(leftBack.getCurrentPosition())
                + Math.abs(rightFront.getCurrentPosition())
                + Math.abs(rightBack.getCurrentPosition())) / 3);
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //1 is right -1 is left
    public void strafe(double direction, double distance, double speed) {
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (averageTicks() * TICKS_TO_INCH_STRAFE < distance) {
            leftBack.setPower(-speed * direction);
            leftFront.setPower(speed * direction);
            rightBack.setPower(speed * direction);
            rightFront.setPower(-speed * direction);

            //    DcMotorEx leftBack, leftFront, rightBack, rightFront;
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    //1 is straight -1 is back
    public void straight(double direction, double distance, double speed) {
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(averageTicks() * TICKS_TO_INCH_FORWARD) < Math.abs(distance)) {
            leftBack.setPower(speed * direction);
            leftFront.setPower(speed * direction);
            rightBack.setPower(speed * direction);
            rightFront.setPower(speed * direction);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void straightWtime(double direction, double speed, double time) {
        timer.reset();
        while (timer.seconds() < time) {
            leftBack.setPower(speed * direction);
            leftFront.setPower(0.5 * speed * direction);
            rightBack.setPower(speed * direction);
            rightFront.setPower(speed * direction);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void strafeWtime(double direction, double speed, double time) {
        timer.reset();
        while (timer.seconds() < time) {
            leftBack.setPower(-speed * direction);
            leftFront.setPower(speed * direction);
            rightBack.setPower(speed * direction);
            rightFront.setPower(-speed * direction);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }


    //1 is right, -1 is left
    public void turnTo(double targetAngle, double speed) {
        double direction;
        if (angleCompare(targetAngle + Math.PI / 2, imu.getAngularOrientation().firstAngle) > angleCompare(targetAngle - Math.PI / 2, imu.getAngularOrientation().firstAngle))
            direction = -1;
        else
            direction = 1;
        while (angleCompare(imu.getAngularOrientation().firstAngle, targetAngle) > 0.02) {
            leftBack.setPower(speed * direction);
            leftFront.setPower(speed * direction);
            rightBack.setPower(-speed * direction);
            rightFront.setPower(-speed * direction);
            telemetry.addData("lb", leftBack.getPower());
            telemetry.addData("lf", leftFront.getPower());
            telemetry.addData("rb", rightBack.getPower());
            telemetry.addData("rf", rightFront.getPower());
            telemetry.update();
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void turnTo(double targetAngle, double speed, int direction) {
        while (angleCompare(imu.getAngularOrientation().firstAngle, targetAngle) > 0.02) {
            leftBack.setPower(speed * direction);
            leftFront.setPower(speed * direction);
            rightBack.setPower(-speed * direction);
            rightFront.setPower(-speed * direction);

        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

//    public void setSlidePosition(int height) {
//        slide.setTargetPosition(height);
//    }
//
//    public void setSlidePower(double power) {
//        if (power == 0) {
//            slide.setPower(slide.getCurrentPosition() / 1000000.0);
//        } else slide.setPower(power);
//    }
//
//    public void moveSlide(double distance, double power) {
//        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while (Math.abs(slide.getCurrentPosition()) < Math.abs(distance * ticksToInchSlide)) {
//            slide.setPower(power);
//        }
//        slide.setPower(0);
//    }
//
//    public void switchSlideMode() {
//        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    //bottom: 0: top around 4181  mid:2985 bottom:1679
//    public void goToJunction(int level) {
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slide.setTargetPosition(levels[level]);
//        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }

//    public void turnToJunction() {
//        double fx = Math.floor(getPoseEstimate().getX() / 24) * 24;
//        double fy = Math.floor(getPoseEstimate().getY() / 24) * 24;
//        double cx = Math.ceil(getPoseEstimate().getX() / 24) * 24;
//        double cy = Math.floor(getPoseEstimate().getY() / 24) * 24;
//        double minAngle = Math.atan2(fx, fy);
//        double minDiff = Math.abs(getPoseEstimate().getHeading() - minAngle);
//        if (Math.abs(Math.atan2(cx, fy) - getPoseEstimate().getHeading()) < minDiff) {
//            minAngle = Math.atan2(cx, fy);
//        } else if (Math.abs(Math.atan2(fx, cy) - getPoseEstimate().getHeading()) < minDiff) {
//            minAngle = Math.atan2(cx, fy);
//        } else if (Math.abs(Math.atan2(cx, cy) - getPoseEstimate().getHeading()) < minDiff) {
//            minAngle = Math.atan2(cx, cy);
//        }
//        turnTo(minAngle, 0.8);
//    }


    //new auton methods

//    public void straightOneOdo(int direction, double speed, double inchesDistance, double targetAngle, double kp, boolean resetEncoder) {
        //forward is negative, back is positive
        //if resetEncoder == true, reset encoder before moving
//        rightOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightOdo.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double initialPos = rightOdo.motor.getCurrentPosition();

//        while (Math.abs(rightOdo.motor.getCurrentPosition()) < Math.abs((inchesDistance * OdoTicksToInchStraight))) {
//            double error = -(targetAngle - imu.getAngularOrientation().firstAngle);
//            leftBack.setPower((speed * direction) - (kp * error));
//            leftFront.setPower((speed * direction) - (kp * error));
//            rightBack.setPower((speed * direction) + (kp * error));
//            rightFront.setPower((speed * direction) + (kp * error));
//        }
//        leftBack.setPower(0);
//        leftFront.setPower(0);
//        rightBack.setPower(0);
//        rightFront.setPower(0);
//    }

//    public void lbEncoderStrafe(double distance, double speed){
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        while (Math.abs(rightBack.getCurrentPosition()) < Math.abs(distance * lbTickstoInch)) {
//            leftBack.setPower(-speed);
//            leftFront.setPower(speed);
//            rightBack.setPower(speed);
//            rightFront.setPower(-speed);
//        }
//        leftBack.setPower(0);
//        leftFront.setPower(0);
//        rightBack.setPower(0);
//        rightFront.setPower(0);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }

//
//    public void straightPID(double speed, double inchesDistance, double targetAngle, double kp, boolean resetEncoder) {
//        //forward is negative, back is positive
//        //if resetEncoder == true, reset encoder before moving
//        rightOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightOdo.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        double initialPos = rightOdo.motor.getCurrentPosition();
//
//        while (Math.abs(rightOdo.motor.getCurrentPosition()) < Math.abs((inchesDistance * OdoTicksToInchStraight))) {
//            double error = -(targetAngle - imu.getAngularOrientation().firstAngle);
//            leftBack.setPower((speed) - (kp * error));
//            leftFront.setPower((speed) - (kp * error));
//            rightBack.setPower((speed) + (kp * error));
//            rightFront.setPower((speed) + (kp * error));
//        }
//        leftBack.setPower(0);
//        leftFront.setPower(0);
//        rightBack.setPower(0);
//        rightFront.setPower(0);
//    }
}