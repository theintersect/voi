package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

import java.text.DecimalFormat;


/**
 * Created by bunnycide on 11/21/16.
 * Drive Test
 */

@TeleOp(name = "Drive Test", group = "Test")

public class DriveTest extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    VOIImu imu;
    BNO055IMU adaImu;
    MecanumDriveTrain driveTrain;
    static double power = 0.5, distance = 60;
    static double changeValue = 0.01;
    double initialAngle;
    public static MecanumDriveTrain.DIRECTION dir = MecanumDriveTrain.DIRECTION.FORWARD;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("Ready", "");
        telemetry.update();
        //setPID();
        waitForStart();
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveTrain.moveLeftNInch(1, 100, 5, true, true, false);
        driveTrain.stopAll();


    }

    public void initialize() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);
        driveTrain = new MecanumDriveTrain(this);

        Servo guide = hardwareMap.servo.get("guide");
        guide.setPosition(ButtonPusherTask.upPosition);
        new ButtonPusherTask(this);
        new IntakeTask(this);
        new CapBallTask(this);
        initialAngle = imu.getAngle();
    }

    public void driveBitMore(int ticks) {
        int br = backRight.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int fl = frontLeft.getCurrentPosition();
        br += ticks;
        bl += ticks;
        fr += ticks;
        fl += ticks;
        driveTrain.driveToPosition(br, bl, fr, fl);
    }


    public void printTicks() {
        System.out.println("br " + backRight.getCurrentPosition());
        System.out.println("bl " + backLeft.getCurrentPosition());
        System.out.println("fr " + frontRight.getCurrentPosition());
        System.out.println("fl " + frontLeft.getCurrentPosition());
    }

    public void chooseDirection() {
        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;
        boolean rBumper = false;
        boolean lBumper = false;
        boolean confirmed = false;
        while (!confirmed) {
            if (gamepad1.a && !aPressed) {
                aPressed = true;
                MecanumDriveTrain.KP += 0.0001;
            }
            if (!gamepad1.a) {
                aPressed = false;
            }
            if (gamepad1.b && !bPressed) {
                bPressed = true;
                MecanumDriveTrain.KP -= 0.0001;
            }
            if (!gamepad1.b) {
                bPressed = false;
            }
            if (gamepad1.x && !xPressed) {
                xPressed = true;
                MecanumDriveTrain.KI += 0.001;
            }
            if (!gamepad1.x) {
                xPressed = false;
            }
            if (gamepad1.y && !yPressed) {
                yPressed = true;
                MecanumDriveTrain.KI -= 0.001;
            }
            if (!gamepad1.y) {
                yPressed = false;
            }
            if (gamepad1.left_bumper && !lBumper) {
                lBumper = true;
                distance -= 1;
            }
            if (!gamepad1.left_bumper) {
                lBumper = false;
            }
            if (gamepad1.right_bumper && !rBumper) {
                rBumper = true;
                distance += 1;
            }
            if (!gamepad1.right_bumper) {
                rBumper = false;
            }
            telemetry.addData("KP", MecanumDriveTrain.KP);
            telemetry.addData("KI", MecanumDriveTrain.KI);
            telemetry.addData("stallTime", MecanumDriveTrain.stallTime);
            telemetry.addData("Choose Direction", dir);
            telemetry.update();
            if (gamepad1.dpad_up) {
                dir = MecanumDriveTrain.DIRECTION.FORWARD;
            } else if (gamepad1.dpad_down) {
                dir = MecanumDriveTrain.DIRECTION.BACKWARD;
            } else if (gamepad1.dpad_left) {
                dir = MecanumDriveTrain.DIRECTION.LEFT;
            } else if (gamepad1.dpad_right) {
                dir = MecanumDriveTrain.DIRECTION.RIGHT;
            }

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                confirmed = true;
            }
        }
        System.out.println(confirmed);
        telemetry.addData("Direction", dir);
        telemetry.addData("Confirmed!", "");
        telemetry.update();
    }

    public void testDrive() {
        switch (dir) {
            case FORWARD:
                driveTrain.moveForwardNInch(power, distance, 10, false, true, false);
                sleep(1000);
                driveTrain.moveBackwardNInch(power, distance, 10, false, true, false);
                break;
            case RIGHT:
                driveTrain.moveRightNInch(power, distance, 10, false, true, true);
                sleep(1000);
                driveTrain.moveLeftNInch(power, distance, 10, false, true, true);
                break;

            case LEFT:
                driveTrain.moveLeftNInch(power, distance, 10, false, true, true);
                sleep(1000);
                driveTrain.moveRightNInch(power, distance, 10, false, true, true);

                break;
            case BACKWARD:
                driveTrain.moveBackwardNInch(power, distance, 10, false, true, false);
                sleep(1000);

                driveTrain.moveForwardNInch(power, distance, 10, false, true, false);

                break;

        }
        printTicks();
        sleep(500);
        printTicks();
    }

    public void chooseSpeed() {

    }

    public void setPID () {
            DecimalFormat df = new DecimalFormat();
            df.setMaximumFractionDigits(8);
            ShooterTest.KMode mode = ShooterTest.KMode.KP;
            boolean confirmed = false;
            boolean rBumper = false;
            boolean lBumper = false;
            boolean upPressed = false;
            boolean downPressed = false;
            while (!confirmed) {
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
                if (gamepad2.a) {
                    mode = ShooterTest.KMode.KP;
                } else if (gamepad2.b) {
                    mode = ShooterTest.KMode.KI;
                } else if (gamepad2.x) {
                    mode = ShooterTest.KMode.KD;
                }

                if (gamepad2.dpad_up && !upPressed) {
                    upPressed = true;
                    switch (mode) {
                        case KP:
                            MecanumDriveTrain.KP += changeValue;
                            break;
                        case KI:
                            MecanumDriveTrain.KI += changeValue;
                            break;
                        case KD:
                            MecanumDriveTrain.KD += changeValue;
                            break;
                    }
                }

                if (gamepad2.dpad_down && !downPressed) {
                    downPressed = true;
                    switch (mode) {
                        case KP:
                            MecanumDriveTrain.KP -= changeValue;
                            break;
                        case KI:
                            MecanumDriveTrain.KI -= changeValue;
                            break;
                        case KD:
                            MecanumDriveTrain.KD -= changeValue;
                            break;
                    }
                }
                if (!gamepad2.dpad_up) {
                    upPressed = false;
                }
                if (!gamepad2.dpad_down) {
                    downPressed = false;
                }
                String kMode = "";
                switch (mode) {
                    case KP:
                        kMode = "KP";
                        break;
                    case KI:
                        kMode = "KI";
                        break;
                    case KD:
                        kMode = "KD";
                        break;
                }
                telemetry.addData("Change value", changeValue);
                telemetry.addData("Current variable", kMode);
                telemetry.addData("KP", df.format(MecanumDriveTrain.KP));
                telemetry.addData("KI", df.format(MecanumDriveTrain.KI));
                telemetry.addData("KD", df.format(MecanumDriveTrain.KD));
                if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                    confirmed = true;
                    telemetry.addData("Confirmed!", "");
                }
                telemetry.update();

            }
            while (gamepad2.left_stick_button && gamepad2.right_stick_button);

    }

}