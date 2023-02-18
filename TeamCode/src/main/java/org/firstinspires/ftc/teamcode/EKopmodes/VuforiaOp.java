package org.firstinspires.ftc.teamcode.EKopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

@Autonomous(name="VuforiaOp", group="auto")
public class VuforiaOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException{
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AZ/jPKD/////AAABmQZtslt8VE5CpeTMoCYH8aosYBRlEThiLP6RnULWCnz/8QoC+/dYzQAio0GN3IyWtleIdvEMp4QVowDaD9NQVed2jQgPei/4hc8OHfPGBBUQnM72Dr0i0XrIvTekdCF0Tny3bVRdSKp2E8kYo9uZp2afQWHhBeO7AYHqp328gFdZOFT/+2ZbLt+hNgrTqm2+z/UKuNkd9cZhJul0oegpzHN9GUa+FNN1hUl4C3ArGzqOq3azeakVLsRbrbCh79KH71a5IbYNLJgMk5tCRkqkVHwJMmQN/6nHnRdPCXWfbuM7xdMCcgzgETDPyeRFALjPQo0KEGeBUGMeZ2bAoQnzOoLmDPbAkru0MnJKo3l+wq4j";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables Images = vuforia.loadTrackablesFromAsset("2023_PowerPlay_navImages");
        Images.get(0).setName("wires");
        Images.get(1).setName("circuitBoard");
        Images.get(2).setName("towers");
        Images.get(3).setName("engine");

        waitForStart();

        Images.activate();

        for(VuforiaTrackable nav : Images){
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) nav.getListener()).getPose();

            if (pose != null){
                VectorF translation = pose.getTranslation();

                telemetry.addData(nav.getName() + " Translation: ", translation);

                double amountToTurn = Math.atan2(translation.get(1), translation.get(2));

                telemetry.addData(nav.getName() + " Degrees to turn: ", Math.toDegrees(amountToTurn));
            }
        }
        telemetry.update();

    }

}
