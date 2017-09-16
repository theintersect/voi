package org.firstinspires.ftc.teamcode.robotutil;

/**
 * Created by Stephen on 10/4/2016.
 * Color sensor with added functionality
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * VOIColorSensorFront.java
 * Class created to encapsulate all Color Sensor
 * functionality and output. Includes averaging multiple readings
 * for accurate output.
 *
 * Created by Stephen on 10/30/2015.
 */


public class VOIColorSensor {

    ColorSensor sensor;
    private ElapsedTime logMessageTimer;
    LinearOpMode opMode;
    public Team team = Team.BLUE;
    public boolean lightOn = false;

    public VOIColorSensor(ColorSensor sensor, LinearOpMode opMode) {
        this.opMode = opMode;
        this.sensor = sensor;
        logMessageTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        logMessageTimer.reset();
    }

    public void enableLED(boolean b)
    {
        sensor.enableLed(b);
    }

    public int getRed()
    {
        return sensor.red();
    }

    public int getBlue()
    {
        return sensor.blue();
    }

    public int getGreen() {
        return sensor.green();
    }

    public int getAlpha()
    {
        return sensor.alpha();
    }

    public void debugOutput(String output) {
        if(logMessageTimer.time() > 50) {
            logMessageTimer.reset();
            System.out.println(output);
        }
    }

    public boolean isWhite() {
        int red =  getRed();
        int blue = getBlue();
        int green = getGreen();
        int i = 0;
        int score = 0;
        logMessageTimer.reset();
        while (i < 10 && opMode.opModeIsActive()){
            if (logMessageTimer.time() > 3){
                i++;
                System.out.println("R: " + sensor.red() + " G: " + sensor.green()+ " B: "  + sensor.blue() );
                logMessageTimer.reset();
                if (sensor.red() >= 3 && sensor.blue() >= 3 && sensor.green() >= 3)
                    score ++;
            }
        }
        return score >= 3;
    }

    public boolean isBlue() {
        int i = 0;
        int score = 0;
        int margin = 4; // 20
        if (lightOn) {
            margin = 10;
        }
        logMessageTimer.reset();
        while (i < 10 && opMode.opModeIsActive()) {
            if (logMessageTimer.time() > 3){
                i++;
                logMessageTimer.reset();
                if (sensor.blue() - sensor.red() >= margin) {
                    score++;
                }
            }
        }
        return score >= 5;
    }

    public boolean isRed() {
        int i = 0;
        int score = 0;
        logMessageTimer.reset();
        int margin = 9; // 19
        if (lightOn) {
            margin = 10;
        }
        while (i < 10 && opMode.opModeIsActive()) {
            if (logMessageTimer.time() > 3){
                i++;
                logMessageTimer.reset();
                if (sensor.red() - sensor.blue() > margin) {
                    score++;
                }
            }
        }
        return score >= 4;
    }

    public boolean correctColor() {
        if (team == Team.BLUE) {
            return isBlue();
        }
        return isRed();
    }

    public boolean wrongColor() {
        if (team == Team.BLUE) {
            return isRed();
        }
        return isBlue();
    }

    public String getColor() {
        return "R: " + getRed() + " G: " + getGreen() + " B: " + getBlue();
    }
}