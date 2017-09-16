package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by bunnycide on 11/17/16.
 * Lift Test
 */
@TeleOp(name = "Lift Test", group = "Test")
@Disabled

public class LiftTest extends LinearOpMode {
    DcMotor capTop, capBottom, frontLeft, frontRight, backLeft, backRight;
    boolean slowDrive = false;
    static double joy1Y, joy2X, joy1X;

    @Override
    public void runOpMode() throws InterruptedException {
        capTop = hardwareMap.dcMotor.get("capTop");
        capBottom = hardwareMap.dcMotor.get("capBottom");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        capTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capTop.setDirection(DcMotorSimple.Direction.REVERSE);
        capBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                capTop.setPower(1);
                capBottom.setPower(capTop.getPower());
            } else if (gamepad1.left_bumper) {
                capTop.setPower(-1);
                capBottom.setPower(capTop.getPower());
            } else {
                capTop.setPower(0);
                capBottom.setPower(capTop.getPower());
            }
            if(slowDrive) {
                joy1Y = -gamepad1.left_stick_y / 2;
                joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y * 3 / 4 : 0;
                joy1X = gamepad1.left_stick_x / 2;
                joy1X = Math.abs(joy1X) > 0.15 ? joy1X * 3 / 4 : 0;
                joy2X = gamepad1.right_stick_x / 2;
                joy2X = Math.abs(joy2X) > 0.15 ? joy2X * 3 / 4 : 0;
            }
            else {
                joy1Y = -gamepad1.left_stick_y;
                joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y * 3 / 4 : 0;
                joy1X = gamepad1.left_stick_x;
                joy1X = Math.abs(joy1X) > 0.15 ? joy1X * 3 / 4 : 0;
                joy2X = gamepad1.right_stick_x;
                joy2X = Math.abs(joy2X) > 0.15 ? joy2X * 3 / 4 : 0;
            }
            frontLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X + joy1X)));
            backLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X - joy1X)));
            frontRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X - joy1X)));
            backRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X + joy1X)));
        }
    }
}