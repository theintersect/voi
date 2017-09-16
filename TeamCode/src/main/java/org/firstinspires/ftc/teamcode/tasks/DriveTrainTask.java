package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 10/15/16.
 */
public class DriveTrainTask extends TaskThread {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    static double joy1X, joy1Y, joy2X;
    int joyStickSign = 1;
    volatile double joyStickMultiplier = 1;
    boolean dpadUpPushed = false;
    boolean xPushed = false;
    public volatile boolean moving = false;
    public volatile boolean aiming = false;

    public double zeroAngle, joyStickAngle, gyroAngle;

    public DriveTrainTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void run() {
        timer.reset();
        while(opMode.opModeIsActive() && running) {
            if (timer.time() > 10) {
                joy1Y = -opMode.gamepad1.left_stick_y;
                joy1X = opMode.gamepad1.left_stick_x;
                joy2X = opMode.gamepad1.right_stick_x;
                if (joy1X < -0.25 && joy1X > -0.7) {
                    joy1X = -0.7;
                } else if (joy1X > 0.25 && joy1X < 0.7) {
                    joy1X = 0.7;
                } else if (Math.abs(joy1X) < 0.25){
                    joy1X = 0;
                }
                if (Math.abs(joy1Y) < 0.25) {
                    joy1Y = 0;
                }

                if (!aiming) {
                    frontLeft.setPower(Math.max(-1, Math.min(1, joyStickMultiplier * (joy1Y + joy2X + joy1X))));
                    backLeft.setPower(Math.max(-1, joyStickMultiplier * Math.min(1, joy1Y + joy2X - joy1X)));
                    frontRight.setPower(Math.max(-1, joyStickMultiplier * Math.min(1, joy1Y - joy2X - joy1X)));
                    backRight.setPower(Math.max(-1, joyStickMultiplier * Math.min(1, joy1Y - joy2X + joy1X)));
                }
                double maxPress = Math.max(Math.abs(joy1Y), Math.abs(joy1X));
                maxPress = Math.max(Math.abs(joy2X), maxPress);
                moving = maxPress > 0.15;
                timer.reset();
            }
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

    }

    public void convertJoyStick(){
        joyStickAngle = Math.atan2(joy1Y, joy1X);
        //System.out.println("Before: " + joy1X + " " + joy1Y);
        double totalAngle = zeroAngle - gyroAngle + joyStickAngle;
        double magnitude = Math.min(1, Math.sqrt(joy1X*joy1X + joy1Y*joy1Y));
        joy1X = Math.cos(totalAngle)*magnitude;
        opMode.telemetry.addData("joy1X",joy1X);
        joy1Y = Math.sin(totalAngle)*magnitude;
        opMode.telemetry.addData("joy1Y", joy1Y);
        opMode.updateTelemetry(opMode.telemetry);
        //System.out.println("\nGyro: " + gyroAngle + " Zero: " + zeroAngle + " Joystick: " + joyStickAngle + "\nTotal: " + totalAngle*180/Math.PI + "\nAfter: " + joy1X + " " + joy1Y);
        System.out.printf("Total: %.4f%n", totalAngle*180/Math.PI);
        System.out.println("Gyro: " + gyroAngle*180/Math.PI);
        System.out.println("Zero: " + zeroAngle *180/Math.PI);
        //System.out.println( "After: " + joy1X + " " + joy1Y);

    }

    @Override
    public void initialize() {
        frontLeft = opMode.hardwareMap.dcMotor.get("frontLeft");
        frontRight = opMode.hardwareMap.dcMotor.get("frontRight");
        backLeft = opMode.hardwareMap.dcMotor.get("backLeft");
        backRight = opMode.hardwareMap.dcMotor.get("backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
