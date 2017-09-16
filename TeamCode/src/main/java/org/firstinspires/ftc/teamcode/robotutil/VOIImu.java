package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Howard on 11/9/16.
 * VOIImu
 */

public class VOIImu {
    public BNO055IMU adafruit;
    private Orientation angles;
    public VOIImu(BNO055IMU adafruit){
        this.adafruit = adafruit;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        adafruit.initialize(parameters);
    }
    public double getAngle(){
        angles = adafruit.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double angle = (AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)));
        angle = -angle;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public double getRadians(){
        return getAngle()*Math.PI/180;
    }

    public double getRoll() {

        angles = adafruit.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        return (double) (AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle)));

    }

    public static double addAngles(double angle1, double angle2){
        double sum = (angle1 + angle2)%360;
        if (sum < 0) {
            sum += 360;
        }
        return sum;
    }

    public static double subtractAngles(double angle1, double angle2){
        double diff1 = (angle1 - angle2);
        int posNeg = 1;
        if (diff1 < 0) {
            posNeg = -1;
            diff1 *= -1;
        }
        if (diff1 > 180) {
            diff1 -= 360;
        }
        return diff1 * posNeg;
    }

    public static double cosine(double angle) {
        angle *= Math.PI/180;
        return Math.cos(angle);
    }

    public static double sine(double angle) {
        angle *= Math.PI/180;
        return Math.sin(angle);
    }

}
