package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Howard on 10/15/16.
 * Button Pusher Task
 */
public class ButtonPusherTask extends TaskThread {

    private CRServo button;
    private Servo guide;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public int pushTime = 450;
    private int outTime = 700;
    public static final double zeroPower = 0;
    public static final double outPower = 1;
    public static final double inPower = -1;
    public static final double upPosition = 0.34;
    public static final double downPosition = 1;
    public volatile boolean teleOp = false;
    private boolean guideDown = false;
    private boolean guidePushed = false;
    private boolean pushButton = false;
    private boolean extendButton = false;
    private boolean withdrawButton = false;

    public ButtonPusherTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void initialize() {
        button = opMode.hardwareMap.crservo.get("button");
        guide = opMode.hardwareMap.servo.get("guide");
        pushTime *= Math.pow(EXPECTED_VOLTAGE / voltage, 2);
        outTime *= Math.pow(EXPECTED_VOLTAGE / voltage, 2);
        button.setPower(0);
        guide.setPosition(upPosition);
    }

    @Override
    public void run() {
        while(opMode.opModeIsActive() && running) {

            // Autonomous
            if (pushButton) {
                pushButton = false;
                buttonPush();
            } else if (extendButton) {
                extendButton = false;
                outPusher();
            } else if (withdrawButton) {
                withdrawButton = false;
                inPusher();
            }

            //TeleOp
            if (teleOp) {
                double rightStick = opMode.gamepad2.right_stick_x;
                if (Math.abs(rightStick) > 0.25) {
                    button.setPower(rightStick);
                    guideDown();
                } else {
                    button.setPower(0);
                }

                if (opMode.gamepad2.left_bumper && !guideDown && !guidePushed) {
                    guideDown();
                    guidePushed = true;
                } else if (opMode.gamepad2.left_bumper && guideDown && !guidePushed){
                    guideUp();
                    guidePushed = true;
                }
                else if (!opMode.gamepad2.left_bumper){
                    guidePushed = false;
                }
            }
        }
        button.setPower(0);

    }

    private void guideDown() {
        guide.setPosition(downPosition);
        guideDown = true;
    }

    private void guideUp() {
        guide.setPosition(upPosition);
        guideDown = false;
    }

    private void buttonPush() {
        timer.reset();
        while (timer.time() < pushTime && opMode.opModeIsActive()) {
            button.setPower(outPower);
        }
        timer.reset();
        while (timer.time() < 300 && opMode.opModeIsActive()) {
            button.setPower(inPower);
        }
        timer.reset();
        while (timer.time() < 200 && opMode.opModeIsActive()) {
            button.setPower(zeroPower);
        }
    }

    private void outPusher() {
        timer.reset();
        while (timer.time() < outTime && opMode.opModeIsActive()) {
            button.setPower(outPower);
        }
        timer.reset();
        while (timer.time() < 1000 && opMode.opModeIsActive()) {
            button.setPower(zeroPower);
        }
    }

    private void inPusher() {
        timer.reset();
        while (timer.time() < outTime + 100 && opMode.opModeIsActive()) {
            button.setPower(inPower);
        }
        timer.reset();
        while (timer.time() < 200 && opMode.opModeIsActive()) {
            button.setPower(zeroPower);
        }
        running = false;
    }

    public void out() {
        extendButton = true;
    }

    public void in() {
        withdrawButton = true;
    }

    public void push() {
        pushButton = true;
    }



}
