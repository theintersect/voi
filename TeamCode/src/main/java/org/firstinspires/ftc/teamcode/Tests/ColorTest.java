package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;

/**
 * Created by Howard on 10/21/16.
 * Color Test to determine that color sensors are working
 */
@TeleOp(name = "Color Test", group = "Test")

public class ColorTest extends LinearOpMode {
    private static final int backID = 0x3a;
    private static final int frontID = 0x3c;
    private static final int intakeID = 0x3e;
    private VOIColorSensor voiFront, voiBack, voiIntake;
    private ColorSensor colorFront, colorBack, colorIntake;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Back:", "Red: " + voiBack.getRed() + " Blue: " + voiBack.getBlue() + " Green " + voiBack.getGreen());
            telemetry.addData("Front: ", "Red: " + voiFront.getRed() + " Blue: " + voiFront.getBlue() + " Green " + voiFront.getGreen());
            telemetry.addData("Intake: ", "Red: " + voiIntake.getRed() + " Blue: " + voiIntake.getBlue() + " Green " + voiIntake.getGreen());

            if (voiBack.isBlue()) {
                telemetry.addData("Back", "Blue");
            }
            if (voiFront.isBlue()) {
                telemetry.addData("Front", "Blue");
            }
            if (voiIntake.isBlue()) {
                telemetry.addData("Intake", "Blue");
            }
            if (voiBack.isRed()) {
                telemetry.addData("Back", "Red");
            }
            if (voiFront.isRed()) {
                telemetry.addData("Front", "Red");
            }
            if (voiIntake.isRed()) {
                telemetry.addData("Intake", "Red");
            }

            telemetry.update();
        }
    }
    public void initialize(){
        colorBack = hardwareMap.colorSensor.get("colorBack");
        colorFront = hardwareMap.colorSensor.get("colorFront");
        colorIntake = hardwareMap.colorSensor.get("colorIntake");
        colorBack.setI2cAddress(I2cAddr.create8bit(backID));
        colorFront.setI2cAddress(I2cAddr.create8bit(frontID));
        colorIntake.setI2cAddress(I2cAddr.create8bit(intakeID));

        voiBack = new VOIColorSensor(colorBack, this);
        voiFront = new VOIColorSensor(colorFront, this);
        voiIntake = new VOIColorSensor(colorIntake, this);
    }
}