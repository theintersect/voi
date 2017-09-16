package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 11/14/16.
 */

public class CapBallTask extends TaskThread {

    private DcMotor capBottom, capTop;
    private Servo forkLeft, forkRight, forkTop;
    double startLeft = 0.55, startRight = 0.10, downLeft = 0.00, downRight = 0.62; // fork lift positions
    double topDown = 0.98, topClamp = 0.25, topUp = 0;
    boolean aPushed = false;
    boolean forkliftOut = false;
    double targetPower = 0;
    public static double holdPower = 0.01;
    public static double KP = 0.000005;
    public static double KD = 0.0001;

    int targetPosition = 0;
    int bottomPosition = 0;
    int topPosition = 0;
    int delta = 75;

    boolean changedMode = false;
    int beforeEnd = 10000;
    boolean pressing = false;
    public static final double flStart = 0.55, frStart = 0.13; // fork left initialize positions
    // decrease flStart and increase frStart to make forklift more out
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer3 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer4 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public CapBallTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
        setForkPosition();
        setLiftPower(0);
    }

    @Override
    public void run() {
        timer2.reset();
        while(opMode.opModeIsActive() && running) {
            // set top servo position
            if (opMode.gamepad2.left_stick_y > 0.8) {
                forkTop.setPosition(topDown);
            } else if (opMode.gamepad2.left_stick_y < -0.8) {
                forkTop.setPosition(topUp);
            } else if (opMode.gamepad2.left_stick_x < -0.8) {
                forkTop.setPosition(topClamp);
            }

            // lift slides
            if (opMode.gamepad1.right_bumper) {
                setLiftPower(1);
                forkliftOut = true;
            } else if (opMode.gamepad1.left_bumper) {
                setLiftPower(-0.5);
            } else if (opMode.gamepad2.dpad_right){
                //holdPosition();
                setLiftPower(0.03);
            } else {
                holdPower = 0.01;
                setLiftPower(0);
            }


            if (opMode.gamepad1.dpad_up && !forkliftOut) {
                setLiftPower(0.5);
                sleep(600);
                setLiftPower(0);
                forkTop.setPosition(topUp);
                sleep(200);
                setLiftPower(-0.5);
                sleep(350);
                setLiftPower(0);
                forkliftOut = true;
                pressing = true;
            }

            if (opMode.gamepad1.dpad_down && forkliftOut) {
                forkliftOut = false;
                foldForkLift();
            }
            if(opMode.gamepad2.right_bumper) {
                forkLeft.setPosition(downLeft);
                forkRight.setPosition(downRight);
                forkliftOut = false;
            } else {
                forkLeft.setPosition(startLeft);
                forkRight.setPosition(startRight);
            }

        }
        setLiftPower(0);
    }

    @Override
    public void initialize()  {
        forkLeft = opMode.hardwareMap.servo.get("forkLeft");
        forkRight = opMode.hardwareMap.servo.get("forkRight");
        forkTop = opMode.hardwareMap.servo.get("forkTop");

        forkTop.setPosition(topDown);
        capBottom = opMode.hardwareMap.dcMotor.get("capBottom");
        capTop = opMode.hardwareMap.dcMotor.get("capTop");
        capBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        capTop.setDirection((DcMotorSimple.Direction.REVERSE));
        bottomPosition = capBottom.getCurrentPosition();
        topPosition = capTop.getCurrentPosition();
    }

    public void setLiftPower(double power) {
        capBottom.setPower(power);
        capTop.setPower(power);
    }

    public void setForkPosition() {
        forkLeft.setPosition(startLeft);
        forkRight.setPosition(startRight);
        forkTop.setPosition(topDown);
        forkliftOut = false;
    }

    void foldForkLift() {
        setLiftPower(-0.5);
        while (opMode.opModeIsActive() && capTop.getCurrentPosition() > 1400) opMode.idle();
        forkLeft.setPosition(downLeft);
        forkRight.setPosition(downRight);
        forkTop.setPosition(topDown);
        while (opMode.opModeIsActive() && capTop.getCurrentPosition() > 200) opMode.idle();
        setLiftPower(0);
        setForkPosition();
    }

    public void setMode(DcMotor.RunMode mode) {
        if(capTop.getMode() != mode) {
            capBottom.setMode(mode);
            capTop.setMode(mode);
        }
    }

    public void setPosition(int pos) {
        capBottom.setTargetPosition(pos);
        capTop.setTargetPosition(pos);
    }

    public void holdPosition() {
        topPosition = capTop.getCurrentPosition();
        bottomPosition = capBottom.getCurrentPosition();
        int range = 3;
        setLiftPower(holdPower * EXPECTED_VOLTAGE / voltage);
        timer3.reset();
        int lastPos = capTop.getCurrentPosition();
        while (opMode.gamepad2.dpad_right && opMode.opModeIsActive()) {
            if (opMode.gamepad1.right_bumper || opMode.gamepad1.left_bumper) {
                return;
            }
            if (timer3.time() > 50) {
                int diff = lastPos - capTop.getCurrentPosition();
                int error = topPosition - capTop.getCurrentPosition();
                holdPower += KP * error + KD * diff;
                setLiftPower(holdPower * EXPECTED_VOLTAGE / voltage);
                timer3.reset();
                lastPos = capTop.getCurrentPosition();
                opMode.telemetry.addData("hold Position", topPosition);
                opMode.telemetry.addData("Current Position", capTop.getCurrentPosition());
                opMode.telemetry.update();
            }
        }
        setLiftPower(0);
    }
}