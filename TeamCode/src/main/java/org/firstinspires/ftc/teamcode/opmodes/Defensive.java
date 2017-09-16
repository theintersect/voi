package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Howard on 5/21/17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Defensive", group = "Drive")

public class Defensive extends LinearOpMode{
    DcMotor frontLeft, frontRight, backLeft, backRight;
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
        long startTime = System.currentTimeMillis();
        driveForward(2);
        while (System.currentTimeMillis() < startTime + 10000);
        driveForward(6);
    }

    void driveForward(double time) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() < start + time * 1000) {
            powerAll(1);
        }
        powerAll(0);
    }

    void powerAll(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
}

