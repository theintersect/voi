package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.VOISweeper;

/**
 * Created by Howard on 10/15/16.
 * Intake Task
 */
public class IntakeTask extends TaskThread {

    public volatile double power = 0;
    public volatile int sweepTime = 0;
    private VOISweeper sweeper;
    public volatile boolean oscillate = false;
    private final int intakeID = 0x3e;
    public final int rejectTime = 600;

    public boolean pidOff = false;
    ColorSensor colorIntake;
    public VOIColorSensor voiColorIntake;
    public FlywheelTask flywheelTask;

    CRServo sweeper1, sweeper2, sweeper3;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime rejectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public boolean visionOn = false;
    public IntakeTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();

    }

    @Override
    public void initialize() {
        colorIntake = opMode.hardwareMap.colorSensor.get("colorIntake");
        colorIntake.setI2cAddress(I2cAddr.create8bit(intakeID));
        voiColorIntake = new VOIColorSensor(colorIntake, opMode);
        voiColorIntake.lightOn = true;

        sweeper1 = opMode.hardwareMap.crservo.get("sweeper1");
        sweeper2 = opMode.hardwareMap.crservo.get("sweeper2");
        sweeper3 = opMode.hardwareMap.crservo.get("sweeper3");
        this.sweeper = new VOISweeper(sweeper1, sweeper2, sweeper3);
        sweeper.setPower(0);
    }

    @Override
    public void run() {
        boolean printed = false;
        boolean print2 = false;
        Thread.currentThread().setPriority(MIN_PRIORITY);
        while(opMode.opModeIsActive() && running) {
            //reject wrong color balls
            if (voiColorIntake.wrongColor()) {
                double initialPower = sweeper1.getPower();
                System.out.println("Wrong Color! E");
                rejectTimer.reset();
                sweeper.setPower(-1);
                opMode.telemetry.addData("Wrong Ball", "!");
                while (opMode.opModeIsActive() && rejectTimer.time() < rejectTime);
                sweeper.setPower(initialPower);
                continue;
            }
            // TeleOp commands

            if (teleOp) {
                if (opMode.gamepad2.dpad_up) {
                    if (flywheelTask.getTargetRate() > 0) {
                        flywheelTask.setPhoneDown();
                    }
                    if (!printed) {
                        System.out.println("Up");
                        printed = true;
                        print2 = false;
                    }
                    sweeper.setPower(1);
                    if (pidOff)
                        flywheelTask.PID_Modify = false;

                } else if (opMode.gamepad2.dpad_down) {
                    if (!printed) {
                        System.out.println("Down");
                        printed = true;
                        print2 = false;
                    }
                    sweeper.setPower(-1);
                } else if (opMode.gamepad1.right_trigger > 0) {
                    sweeper.setPower(1);
                    if (pidOff)
                        flywheelTask.PID_Modify = false;
                } else if (opMode.gamepad1.left_trigger > 0) {
                    sweeper.setPower(-1);
                } else {
                    sweeper.setPower(0);
                    if (!print2) {
                        System.out.println("None");
                        print2 = true;
                        printed = false;
                    }
                    if (pidOff)
                        flywheelTask.PID_Modify = true;

                }
            } else {
                if (visionOn && voiColorIntake.correctColor()) {
                    sleep(500);
                    //while (voiColorIntake.correctColor() && opMode.opModeIsActive());
                    sweeper.setPower(0);
                }
            }
            // Autonomous commands
            if (power != 0) {
                sweeper.setPower(power);
                power = 0;
                int temp = sweepTime;
                sweepTime = 0;
                sleep(temp);
                sweeper.setPower(0);
            }
            if (oscillate) {
                boolean stoppedPressing = false;
                double pow = 1;
                sweeper.setPower(pow);
                timer.reset();
                while (oscillate && opMode.opModeIsActive()) {
                    if (timer.time() > 25) {
                        pow = -pow;
                        sweeper.setPower(pow);
                        timer.reset();
                    }
                }
            }


        }
        sweeper.setPower(0);
    }

    public void setPower(double power) {
        sweeper.setPower(power);
    }

    @Override
    public void sleep (int ms) {
        ElapsedTime sleepTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        sleepTimer.reset();
        while (opMode.opModeIsActive() && sleepTimer.time() < ms) {
            if (voiColorIntake.wrongColor()) {
                sweeper.setPower(-1);
                rejectTimer.reset();
                while (opMode.opModeIsActive() && timer.time() < rejectTime);
            }
        }
    }

    public boolean correctColor() {
        return voiColorIntake.correctColor();
    }

    public boolean wrongColor() {
        return voiColorIntake.wrongColor();
    }

    public void setTeam(Team team) {
        voiColorIntake.team = team;
    }

}
