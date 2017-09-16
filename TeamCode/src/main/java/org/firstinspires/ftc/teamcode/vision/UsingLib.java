package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.lasarobotics.vision.opmode.ManualVisionOpMode;
import org.opencv.core.Mat;

/**
 * Created by Stephen on 12/24/2016.
 * Using Lib
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "UsingLib", group = "Tests")
@Disabled

public class UsingLib extends ManualVisionOpMode {
    public Mat frame(Mat rgba, Mat gray) {
        return rgba;
    }
}
