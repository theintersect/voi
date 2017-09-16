package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Howard on 10/1/16.
 * Outreach Drive Train
 */
@TeleOp(name = "Outreach Drive", group = "Drive")
@Disabled

public class OutreachDrive extends LinearOpMode {

    DcMotor backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            double joy1Y = gamepad1.left_stick_y/2;
            double joy2Y = gamepad1.right_stick_y/2;
            backLeft.setPower(joy1Y);
            backRight.setPower(joy2Y);
        }
    }
}
