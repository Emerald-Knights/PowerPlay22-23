package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot extends MecanumDrive {


    public boolean RUN_USING_ENCODER;


    DcMotorEx leftBack, leftFront, rightBack, rightFront;

    BNO055IMU imu;
    Orientation currentAngle;

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;

    public final int DIRECTION = 1;
    final static double TICKS_TO_INCH_FORWARD = 37.87;
    final static double TICKS_TO_INCH_STRAFE = 70.68;
    static DcMotor[] encoderMotors;

    public Robot(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderMotors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.getAngularOrientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        currentAngle = imu.getAngularOrientation();
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
    }


    public void resetEncoders(){
//        for (int i = 0; i < encoderMotors.length; i++){
//            encoderMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            encoderMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initOpenCV(HardwareMap hardwareMap) {
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
}
