package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Stephen on 9/11/2016.
 */

@TeleOp(name = "Mecanum Drive", group = "Drive")

public class MecanumDrive extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    final double maxPower = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            double joy1Y = -gamepad1.left_stick_y;
            joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
            double joy1X = gamepad1.left_stick_x;
            joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
            double joy2X = gamepad1.right_stick_x;
            joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;
            frontLeft.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
            backLeft.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
            frontRight.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
            backRight.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));
        }
    }
}
