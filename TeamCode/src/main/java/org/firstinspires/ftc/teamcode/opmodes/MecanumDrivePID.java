package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by Stephen on 9/11/2016.
 * Mecanum Drive PID
 */

@TeleOp(name = "Mecanum Drive PID", group = "Tests")

public class MecanumDrivePID extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    VOIImu imu;
    BNO055IMU adaImu;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double output;

    static double KP = 0.001;
    static double KD = 0.001;
    double KI = 0;
    double I = 0;
    double D;
    double P;
    double prevErr = 0;

    @Override
    public void runOpMode() {
        initialize();
        telemetry.addData("Ready!", "");
        telemetry.update();
        waitForStart();
        timer.reset();

        double joy1Y, joy1X, joy2X;
        double base = imu.getAngle();
        boolean aPressed = false;
        boolean bPressed = false;
        while(opModeIsActive()) {
            if (gamepad1.a && !aPressed) {
                aPressed = true;
                KP += 0.001;
            }
            if (!gamepad1.a) {
                aPressed = false;
            }
            if (gamepad1.b && !bPressed) {
                bPressed = true;
                KP -= 0.001;
            }
            if (!gamepad1.b) {
                bPressed = false;
            }
            if (timer.time() > 10) {
                double err = VOIImu.subtractAngles(imu.getAngle(), base);



                joy1Y = 0;
                joy1X = gamepad1.left_stick_x * 3/4;
                joy2X = gamepad1.right_stick_x * 3/4;
                if (joy1X != 0) {

                    double delta = 0;
                    P = err * KP;
                    I += err * delta * KI;
                    if (delta == 0) {
                        delta = 1;
                    }
                    D = (err - prevErr) / delta * KD;
                    D = 0;
                    output = -P + I + D;
                    //output = 0;
                    frontLeft.setPower(Math.max(-1, Math.min(1, (joy1Y + joy2X + joy1X - output))));
                    backLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X - joy1X + output)));
                    frontRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X - joy1X + output)));
                    backRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X + joy1X - output)));
                } else {
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                }
                telemetry.addData("KP", KP);
                telemetry.addData("Error", err);
                telemetry.addData("output", output);
                telemetry.addData("FL", frontLeft.getPower());
                telemetry.addData("FR", frontRight.getPower());
                telemetry.addData("BL", backLeft.getPower());
                telemetry.addData("BR", backRight.getPower());
                telemetry.update();
                timer.reset();

            }

        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
//        while(opModeIsActive()) {
//

//

//            frontLeft.setPower(Math.max(-1, Math.min(1, (joy1Y + joy2X + joy1X))));
//            backLeft.setPower(Math.max(-1, Math.min(1, joy1Y + joy2X - joy1X)));
//            frontRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X - joy1X)));
//            backRight.setPower(Math.max(-1, Math.min(1, joy1Y - joy2X + joy1X)));
//            if(joy1X != 0) {
//
//                double KP = 0.1;
//                double KD = 0.001;
//                double KI = 0;
//                double I = 0;
//                double D;
//                double P;
//                double prevErr = 0;
//                base = imu.getAngle();
//                while(opModeIsActive() && (Math.abs(joy1X) > 0 || Math.abs(joy1Y) > 0) && joy2X == 0) {
//                    if (timer.time() > 50) {
//                        //Update joystick to make sure that the action is still occurring
//                        joy1Y = -gamepad1.left_stick_y;
//                        joy1X = gamepad1.left_stick_x;
//                        joy2X = gamepad1.right_stick_x;
//

//                        frontLeft.setPower(crange(frontLeft.getPower() - P));
//                        backLeft.setPower(crange(backLeft.getPower() + P));
//                        frontRight.setPower(crange(frontRight.getPower() + P));
//                        backRight.setPower(crange(backRight.getPower() - P));*/
//                        System.out.println("FL: " + (joy1Y + joy1X - P) + " BL: " + (joy1Y - joy1X - P) + " FR: " + (joy1Y - joy1X + P) + " BR: " + (joy1Y + joy1X + P));
//                        timer.reset();
//                    }
//                }
//
//            } /*else if(Math.abs(joy2X) > 0) {
//                frontLeft.setPower(crange(joy1Y + joy2X + joy1X));
//                backLeft.setPower(crange(joy1Y + joy2X - joy1X));
//                frontRight.setPower(crange(joy1Y - joy2X - joy1X));
//                backRight.setPower(crange(joy1Y - joy2X + joy1X));
//            } else {
//                frontLeft.setPower(0);
//                backLeft.setPower(0);
//                frontRight.setPower(0);
//                backRight.setPower(0);
//            }*/

    }

    private double crange(double x) {
        return Math.min(1, Math.max(-1, x));
    }

    public void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        CRServo button = hardwareMap.crservo.get("button");
        button.setPower(0);
        IntakeTask intakeTask = new IntakeTask(this);
        CapBallTask capBallTask = new CapBallTask(this);
    }

}
