package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

import java.text.DecimalFormat;

/**
 * Created by Howard on 1/17/17.
 * Cap Power PID calibration
 */
@TeleOp(name = "Cap power", group = "Test")

public class CapPower extends LinearOpMode {
    CapBallTask capBallTask;
    IntakeTask intakeTask;
    ButtonPusherTask buttonPusherTask;
    DecimalFormat df = new DecimalFormat();
    public static double changeValue = 0.001;

    public void runOpMode() {
        df.setMaximumFractionDigits(6);
        initialize();
        changePower();
        waitForStart();
        capBallTask.start();
        while (opModeIsActive());
    }
    public void initialize() {
        capBallTask = new CapBallTask(this);
        intakeTask = new IntakeTask(this);
        buttonPusherTask = new ButtonPusherTask(this);
    }

    public void changePower () {
        boolean confirmed = false;
        boolean aPressed = false, bPressed = false, xPressed = false, yPressed = false;
        boolean upPressed = false, downPressed = false;
        boolean rBumper = false, lBumper = false;
        while (!confirmed) {

            if (gamepad2.a && !aPressed) {
                aPressed = true;
                CapBallTask.holdPower += changeValue;
            }
            if (!gamepad2.a) {
                aPressed = false;
            }
            if (gamepad2.b && !bPressed) {
                bPressed = true;
                CapBallTask.holdPower -= changeValue;
            }
            if (!gamepad2.b) {
                bPressed = false;
            }
            if (gamepad2.x && !xPressed) {
                xPressed = true;
                CapBallTask.KP += changeValue;
            }
            if (!gamepad2.x) {
                xPressed = false;
            }
            if (gamepad2.y && !yPressed) {
                yPressed = true;
                CapBallTask.KP -= changeValue;
            }
            if (!gamepad2.y) {
                yPressed = false;
            }
            if (gamepad2.dpad_up && !upPressed) {
                upPressed = true;
                CapBallTask.KD += changeValue;
            }
            if (!gamepad2.dpad_up) {
                upPressed = false;
            }
            if (gamepad2.dpad_down && !downPressed) {
                downPressed = true;
                CapBallTask.KD -= changeValue;
            }
            if (!gamepad2.dpad_down) {
                downPressed = false;
            }
            if (gamepad2.right_bumper && !rBumper) {
                rBumper = true;
                changeValue *= 10;
            }
            if (gamepad2.left_bumper && !lBumper) {
                lBumper = true;
                changeValue /= 10;
            }
            if (!gamepad2.right_bumper) {
                rBumper = false;
            }
            if (!gamepad2.left_bumper) {
                lBumper = false;
            }
            telemetry.addData("changeValue", df.format(changeValue));
            telemetry.addData("holdPower", df.format(CapBallTask.holdPower));
            telemetry.addData("KP", df.format(CapBallTask.KP));
            telemetry.addData("KD", df.format(CapBallTask.KD));
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                confirmed = true;
                telemetry.addData("Confirmed!", "");
            }
            telemetry.update();
        }
    }

}
