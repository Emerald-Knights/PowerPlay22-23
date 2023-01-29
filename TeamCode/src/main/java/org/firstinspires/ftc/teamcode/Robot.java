package org.firstinspires.ftc.teamcode;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot extends SampleMecanumDrive {

    public boolean RUN_USING_ENCODER;
    private boolean clawClosed = false;

    public DcMotorEx leftBack, leftFront, rightBack, rightFront;
    DcMotor test;
    public Servo leftClaw;
    public Servo rightClaw;
    DcMotor slide1, slide2;

    public DistanceSensor distance;
    public BNO055IMU imu;
    Orientation currentAngle;
    ElapsedTime timer;

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;

    public final int DIRECTION = 1;
    final static double TICKS_TO_INCH_FORWARD = 0.0265;
    final static double TICKS_TO_INCH_STRAFE = 0.01975;
    static DcMotor[] encoderMotors;

    PIDController slidePID;
    int currSlidePosition = 0;
    public int targetSlidePosition = 0;
    int[] slidePosition = new int[]{0, 0, 0, 0};
    InterpLUT maxVelLut = new InterpLUT();
    double maxVel = 1;

    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        super(hardwareMap);
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //slide1 = hardwareMap.get(DcMotor.class, "slide1");
        //slide2 = hardwareMap.get(DcMotor.class, "slide2");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        //test = hardwareMap.get(DcMotor.class, "test");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        //leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderMotors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};

//        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide1.setDirection(DcMotorEx.Direction.REVERSE);
//        slide2.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
        timer = new ElapsedTime();

        //constants to tune
        slidePID = new PIDController(0, 0, 0);
        maxVelLut.add(0, 1);
        maxVelLut.add(10, 1);
    }

    public void initOpenCV() {
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);
        webcam.setPipeline(new DetectorPipeline());
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        OpenCvWebcam finalWebcam = webcam;
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

    //teleop methods
    public void moveClaw() {
        if(!clawClosed) {
            rightClaw.setPosition(0.365);
            leftClaw.setPosition(0.07);
        } else {
            rightClaw.setPosition(0.23);
            leftClaw.setPosition(0.23);
        }
        clawClosed = !clawClosed;
    }

    //auton methods
    public static double angleWrap(double angle){
        while(angle>Math.PI){
            angle-=2*Math.PI;
        }
        while(angle<-Math.PI){
            angle+=2*Math.PI;
        }
        return angle;
    }

    public static double factorial(int num){
        int product = 1;
        for (int i = num; i >= 1; i--){
            product *= i;
        }
        return product;
    }

    public static double sin(double angle){
        double sum = 0;
        for(int i = 1; i<17; i+=4){
            sum += (Math.pow(angle, i)/factorial(i)) - (Math.pow(angle, i+2)/factorial(i+2));
        }
        return(sum);
    }

    public static double cos(double angle){
        double sum = 0;
        for(int i = 0; i<16; i+=4){
            sum += (Math.pow(angle, i)/factorial(i)) - (Math.pow(angle, i+2)/factorial(i+2));
        }
        return(sum);
    }

    public int averageTicks() {
        return (Math.abs(leftFront.getCurrentPosition())
                + Math.abs(leftBack.getCurrentPosition())
                + Math.abs(rightFront.getCurrentPosition())
                + Math.abs(rightBack.getCurrentPosition()))/4;
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
        while(averageTicks() * TICKS_TO_INCH_STRAFE < distance){
            leftBack.setPower(-speed*direction);
            leftFront.setPower(speed*direction);
            rightBack.setPower(speed*direction);
            rightFront.setPower(-speed*direction);

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

        while(averageTicks() * TICKS_TO_INCH_FORWARD < distance){
            leftBack.setPower(speed*direction);
            leftFront.setPower(speed*direction);
            rightBack.setPower(speed*direction);
            rightFront.setPower(speed*direction);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //1 is right, -1 is left
    public void turnTo(double targetAngle, double speed) {
        double direction;
        if(targetAngle > imu.getAngularOrientation().firstAngle && targetAngle < angleWrap(imu.getAngularOrientation().firstAngle + Math.PI)) direction = 1;
        else direction = -1;
        while(angleWrap(Math.abs(imu.getAngularOrientation().firstAngle - targetAngle)) < 0.03){
            leftBack.setPower(speed*direction);
            leftFront.setPower(speed*direction);
            rightBack.setPower(-speed*direction);
            rightFront.setPower(-speed*direction);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveSlide(double vector, double time) {
        timer.reset();
        while(timer.seconds() < time) {
            slide1.setPower(vector);
            slide2.setPower(vector);
        }
        slide1.setPower(0);
        slide2.setPower(0);
    }

    public boolean slideUpdate() {
        float target = slidePosition[targetSlidePosition];
        maxVel = maxVelLut.get(slide1.getCurrentPosition());
        if ((target - slide1.getCurrentPosition()) > maxVel){
            double pow = slidePID.logUpdate(slide1.getCurrentPosition() + maxVel, slide1.getCurrentPosition(), this.linearOpMode.telemetry, "slide");
            if(pow < 0.8) {
                slide1.setPower(pow);
                slide2.setPower(pow);
            }
            return false;
        }
        else {
            currSlidePosition = targetSlidePosition;
            slide1.setPower(0);
            slide2.setPower(0);
            return true;
        }
    }

    public void turnToJunction() {
        double fx = Math.floor(getPoseEstimate().getX()/24) * 24;
        double fy = Math.floor(getPoseEstimate().getY()/24) * 24;
        double cx = Math.ceil(getPoseEstimate().getX()/24) * 24;
        double cy = Math.floor(getPoseEstimate().getY()/24) * 24;
        double minAngle = Math.atan2(fx, fy);
        double minDiff = Math.abs(getPoseEstimate().getHeading() - minAngle);
        if(Math.abs(Math.atan2(cx, fy) - getPoseEstimate().getHeading()) < minDiff) {
            minAngle = Math.atan2(cx, fy);
        } else if(Math.abs(Math.atan2(fx, cy) - getPoseEstimate().getHeading()) < minDiff) {
            minAngle = Math.atan2(cx, fy);
        } else if(Math.abs(Math.atan2(cx, cy) - getPoseEstimate().getHeading()) < minDiff) {
            minAngle = Math.atan2(cx, cy);
        }
        turnTo(minAngle, 0.8);
    }
}
