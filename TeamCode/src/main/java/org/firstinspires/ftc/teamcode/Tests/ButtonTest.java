package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;

/**
 * Created by Howard on 1/8/17.
 * Button pusher test
 */

@TeleOp(name = "Button Test", group = "Test")


public class ButtonTest extends LinearOpMode {

    private CRServo button;
    private ButtonPusherTask buttonPusherTask;
    private static int pushTime = 500;
    private Servo guide;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //choosePower();
        waitForStart();
        buttonPusherTask.start();
        guide.setPosition(ButtonPusherTask.downPosition);
        sleep(2000);
    }

    private void initialize() {
        button = hardwareMap.crservo.get("button");
        button.setPower(ButtonPusherTask.zeroPower);
        new CapBallTask(this);
        new IntakeTask(this);
        buttonPusherTask = new ButtonPusherTask(this);
        guide = hardwareMap.servo.get("guide");
        guide.setPosition(ButtonPusherTask.upPosition);
    }

    private void choosePower() {
        boolean upPressed = false, downPressed = false;
        boolean confirmed = false;
        while (!confirmed) {
            if (gamepad1.dpad_up && !upPressed) {
                pushTime += 50;
                upPressed = true;
            }
            if (gamepad1.dpad_down && !downPressed) {
                pushTime -= 50;
                downPressed = true;
            }
            if (!gamepad1.dpad_down) {
                downPressed = false;
            }
            if (!gamepad1.dpad_up) {
                upPressed = false;
            }
            telemetry.addData("pushTime", pushTime);
            telemetry.update();
            confirmed = gamepad1.left_stick_button && gamepad1.right_stick_button;
        }
    }
}