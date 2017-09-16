package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;

@TeleOp(name = "MotorTest")
@Disabled
/**
 * Created by Howard on 2/5/17.
 * Motor Test
 */

public class MotorTest extends LinearOpMode {
    private DcMotor flywheelRight;
    private DcMotor flywheelLeft;
    public volatile FlywheelTask.FlywheelState state;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static double power = 1;

    public void runOpMode() {
        initialize();
        choosePower();
        waitForStart();
        flywheelLeft.setPower(power);
        sleep(1000);
        int before = flywheelLeft.getCurrentPosition();
        sleep(5000);
        System.out.println((double)(flywheelLeft.getCurrentPosition() - before)/5000);
    }

    public void initialize() {
        flywheelLeft = hardwareMap.dcMotor.get("flywheelLeft");
        flywheelRight = hardwareMap.dcMotor.get("flywheelRight");
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void choosePower() {
        boolean confirmed = false;
        boolean upPressed = false;
        boolean downPressed = false;
        while (!confirmed) {
            if (gamepad1.dpad_up && !upPressed) {
                upPressed = true;
                power += 0.01;
            }
            if (gamepad1.dpad_down && !downPressed) {
                downPressed = true;
                power -= 0.01;
            }
            if (!gamepad1.dpad_up) {
                upPressed = false;
            }
            if (!gamepad1.dpad_down) {
                downPressed = false;
            }
            telemetry.addData("power", power);
            telemetry.update();
            confirmed = gamepad1.left_stick_button && gamepad1.right_stick_button;
        }
    }
}
