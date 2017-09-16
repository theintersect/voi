package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Howard on 2/25/17.
 */

@TeleOp(name = "Single Flywheel Test", group = "Tests")

public class SingleFlywheelTest extends LinearOpMode {
    DcMotor flywheelTop, flywheelBottom;

    static double pow = 0.6;
    @Override
    public void runOpMode() {
        setPowers();
        initRobot();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                powerFlywheel(pow);
            } else {
                powerFlywheel(0);
            }
        }
    }

    public void initRobot() {
        flywheelTop = hardwareMap.dcMotor.get("flywheelTop");
        flywheelBottom = hardwareMap.dcMotor.get("flywheelBottom");
        flywheelTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelTop.setDirection(REVERSE);
        flywheelBottom.setDirection(REVERSE);
    }

    public void powerFlywheel(double power) {
        flywheelTop.setPower(power);
        flywheelBottom.setPower(power);
    }

    public void setPowers() {
        boolean confirmed = false;
        boolean upPressed = false;
        boolean downPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;

        while (!confirmed) {
            if (gamepad1.dpad_up && !upPressed) {
                upPressed = true;
                pow += 0.01;
            }
            if (gamepad2.dpad_down && !downPressed) {
                downPressed = true;
                pow -= 0.01;
            }
            if (!gamepad1.dpad_up) {
                upPressed = false;
            }
            if (!gamepad1.dpad_down) {
                downPressed = false;
            }

            telemetry.addData("Power", pow);
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                confirmed = true;
                telemetry.addData("Confirmed!", "");
            }
            telemetry.update();


        }
    }
}
