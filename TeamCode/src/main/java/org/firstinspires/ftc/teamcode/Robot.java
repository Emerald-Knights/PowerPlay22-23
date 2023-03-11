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

public class Robot extends SampleMecanumDrive {

    public boolean RUN_USING_ENCODER;
    private boolean clawClosed = false;
    private boolean neckFront = false;
    private boolean rnpUp = false;

    public DcMotorEx leftBack, leftFront, rightBack, rightFront;
    DcMotor test;
    public Servo leftClaw;
    public Servo rightClaw;
    public Servo neck;
    public DcMotor slide;

    public DistanceSensor distance;
    public BNO055IMU imu;
    Orientation currentAngle;
    ElapsedTime timer;

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Encoder leftOdo, centerOdo, rightOdo;

    public final int DIRECTION = 1;
    final static double TICKS_TO_INCH_FORWARD = 0.0265;
    final static double TICKS_TO_INCH_STRAFE = 0.01975;

    final static double OdoTicksToInchStraight = 467.5;
    final static double OdoTicksToInchStrafe = 467.2;


    //theoretical ticks to inch 32.7404454359 (360 / circumference of the wheel)

    public int[] slidePosition = new int[3];
    public int[] levels = new int[]{0, 1800, 3019, 4240};
    InterpLUT slideZeroPower = new InterpLUT();

    public OpenCvWebcam webcam;

    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        super(hardwareMap);
        leftOdo = new Encoder(hardwareMap.get(DcMotorEx.class, "leftOdo"));
        centerOdo = new Encoder(hardwareMap.get(DcMotorEx.class, "centerOdo"));
        rightOdo = new Encoder(hardwareMap.get(DcMotorEx.class, "rightOdo"));
        leftOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdo.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOdo.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        neck = hardwareMap.get(Servo.class, "neck");

        distance = hardwareMap.get(DistanceSensor.class, "distance");
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
        timer = new ElapsedTime();

        //add slide data points: slideZeroPower.add();

        //slideZeroPower.createLUT();
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
        setPoseEstimate(new Pose2d(x, y, imu.getAngularOrientation().firstAngle));
    }

    //teleop methods
    public void moveClaw() {
        if (clawClosed) {
            //open position
            rightClaw.setPosition(0.64);
            leftClaw.setPosition(0.68);
        } else {
            //closed position
            rightClaw.setPosition(0.50);
            leftClaw.setPosition(0.85);
        }
        clawClosed = !clawClosed;
    }

    public void moveNeck() {
        if (neckFront) {
            //back positoin
            neck.setPosition(1);
        } else {
            neck.setPosition(0.15);
        }
        neckFront = !neckFront;
    }


    public void overextendClaw() {
        rightClaw.setPosition(0.7);
        leftClaw.setPosition(0);
        clawClosed = false;
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
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSlidePosition(int height) {
        slide.setTargetPosition(height);
    }

    public void setSlidePower(double power) {
        if (power == 0) {
            slide.setPower(slide.getCurrentPosition() / 1000000.0);
        } else slide.setPower(power);
    }

    public void moveSlide(double power, double time) {

    }

    public void switchSlideMode() {
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //bottom: 0: top around 4181  mid:2985 bottom:1679
    public void goToJunction(int level) {
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setTargetPosition(levels[level]);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turnToJunction() {
        double fx = Math.floor(getPoseEstimate().getX() / 24) * 24;
        double fy = Math.floor(getPoseEstimate().getY() / 24) * 24;
        double cx = Math.ceil(getPoseEstimate().getX() / 24) * 24;
        double cy = Math.floor(getPoseEstimate().getY() / 24) * 24;
        double minAngle = Math.atan2(fx, fy);
        double minDiff = Math.abs(getPoseEstimate().getHeading() - minAngle);
        if (Math.abs(Math.atan2(cx, fy) - getPoseEstimate().getHeading()) < minDiff) {
            minAngle = Math.atan2(cx, fy);
        } else if (Math.abs(Math.atan2(fx, cy) - getPoseEstimate().getHeading()) < minDiff) {
            minAngle = Math.atan2(cx, fy);
        } else if (Math.abs(Math.atan2(cx, cy) - getPoseEstimate().getHeading()) < minDiff) {
            minAngle = Math.atan2(cx, cy);
        }
        turnTo(minAngle, 0.8);
    }


    //new auton methods

    public void straightOneOdo(int direction, double speed, double inchesDistance, boolean resetEncoder) {
        //forward is negative, back is positive
        //if resetEncoder == true, reset encoder before moving
        if (resetEncoder) {
            leftOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            centerOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        while (rightOdo.getCurrentPosition() < (inchesDistance * OdoTicksToInchStraight)){
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

    public void strafeOdoEncoder(double direction, double speed, double distance, boolean encoderReset){
        if(encoderReset){
            centerOdo.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        while(centerOdo.getCurrentPosition() < distance * OdoTicksToInchStrafe){
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
}

