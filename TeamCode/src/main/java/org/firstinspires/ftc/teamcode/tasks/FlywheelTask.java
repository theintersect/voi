package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.tasks.FlywheelTask.FlywheelState.STATE_ACCELERATING;
import static org.firstinspires.ftc.teamcode.tasks.FlywheelTask.FlywheelState.STATE_ADJUSTING;
import static org.firstinspires.ftc.teamcode.tasks.FlywheelTask.FlywheelState.STATE_RUNNING_NEAR_TARGET;
import static org.firstinspires.ftc.teamcode.tasks.FlywheelTask.FlywheelState.STATE_STOPPED;

/**
 * Created by Howard on 10/15/16.
 * FlywheelTask
 */
public class FlywheelTask extends TaskThread {

    Servo phoneServo;
    private DcMotor flywheelRight;
    private DcMotor flywheelLeft;
    public volatile FlywheelState state;
    private final int THEORETICAL_MAX_RPM = 1800;
    private final int FULL_SPEED_RPM = 1300;
    private final double MAXPOWER = 0.42;
    private final int TICKS_PER_REV = 112;
    private final double MAX_ENCODER_TICKS_PER_MS = 2.9;
    private final double MAX_ALLOWED_ERROR = 0.05;      //When the difference between the actual speed and targeted speed is smaller than this percentage, the state will display as RUNNNING_NEAR_TARGET.
    private final double CLOSE_ERROR = 0.06;
    public double currentErrorLeft, currentErrorRight;
    public static double KP = 0.08;     //0.004
    public static double KI = 0;        //0
    public static double KD = 0.001;    //0.0004

    static final double visionPosition = 0.2, downPosition = 0, restPosition = 0.68;

    double voltageRatio;
    public static double lowPow = 0.66;
    public static double highPow = 0.72;
    private double targetEncoderRate = 0;
    private int lastEncoderReadingLeft = 0;
    private int lastEncoderReadingRight = 0;
    private double leftPower = 0;
    private double rightPower = 0;
    public boolean PID_Modify = false;
    public volatile int count = 1;

    public static final int interval = 300;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double totalErrorRight = 0;
    double totalErrorLeft = 0;
    double prevErrorR = 0;
    double prevErrorL = 0;

    DecimalFormat df = new DecimalFormat();

    public volatile int goodTimes = 0;

    public enum FlywheelState {
        STATE_STOPPED, STATE_ACCELERATING, STATE_ADJUSTING, STATE_RUNNING_NEAR_TARGET
    }

    public FlywheelTask(LinearOpMode opMode) {
        this.opMode = opMode;
        initialize();
    }

    @Override
    public void initialize() {
        phoneServo = opMode.hardwareMap.servo.get("phoneServo");
        phoneServo.setPosition(restPosition);
        flywheelRight = opMode.hardwareMap.dcMotor.get("flywheelRight");
        flywheelLeft = opMode.hardwareMap.dcMotor.get("flywheelLeft");
        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        PID_Modify = true;
    }

    @Override
    public void run() {
        timer.reset();
        df.setMaximumFractionDigits(4);
        while(opMode.opModeIsActive() && running) {
            if (teleOp) {
                boolean press = true;
                if (opMode.gamepad2.a) {
                    setFlywheelPow(lowPow);
                } else if (opMode.gamepad2.x) {
                    setFlywheelPow(0);
                } else if (opMode.gamepad2.b) {
                    setFlywheelPow(highPow);
                } else if (opMode.gamepad2.y) {
                    setFlywheelPow(-0.4);
                } else if (opMode.gamepad1.a) {
                    setFlywheelPow(lowPow);
                } else if (opMode.gamepad1.x) {
                    setFlywheelPow(0);
                } else if (opMode.gamepad1.b) {
                    setFlywheelPow(highPow);
                } else if (opMode.gamepad1.y) {
                    setFlywheelPow(-0.4);
                }
                if (press) {
                    goodTimes = 0;
                }
            }
            if (PID_Modify) {
                pidCalibration();
            }
        }
        flywheelRight.setPower(0);
        flywheelLeft.setPower(0);
    }

    private double range(double rawPower) {
        if(rawPower > 1) {
            System.out.println("Raw power going over 1, tune your code better");
            return 1;
        } else if (rawPower < -1) {
            System.out.println("Raw power going under 1, something is seriously wrong");
            return -1;
        } else {
            return rawPower;
        }
    }

    public void setFlywheelPow(double power) {
        setFlywheelPow(power, true);
    }

    public void setFlywheelPow(double power, boolean setPow) {

        if (timer2.time() > 400) {
            System.out.println("Changed Power " + power);
            if (power > 0) {
                //flywheelLeft.setPower(1.2*power);
                //flywheelRight.setPower(1.2*power);
            }
            timer.reset();
            voltageRatio = EXPECTED_VOLTAGE / voltage;
            targetEncoderRate = (MAX_ENCODER_TICKS_PER_MS * power);
            if (setPow) {
                if (power == 0) {
                    state = STATE_STOPPED;
                } else {
                    state = STATE_ACCELERATING;
                }
                leftPower = rightPower = power * MAXPOWER * voltageRatio;
                updatePowers();
            }
            //We intentionally set the power so that it is highly likely to be lower than the
            //"correct" value so that it continues to adjust upwards.
            lastEncoderReadingRight = 0;
            lastEncoderReadingLeft = 0;
            timer2.reset();
        }
        //flywheelLeft.setPower(power);
        //flywheelRight.setPower(power);
    }

    private void updatePowers() {
        flywheelRight.setPower(rightPower);
        flywheelLeft.setPower(leftPower);
    }

    private int getEncoderLeft() {
        return flywheelLeft.getCurrentPosition();
    }

    private int getEncoderRight() {
        return flywheelRight.getCurrentPosition();
    }

    public double getTargetRate() {
        return targetEncoderRate;
    }

    public FlywheelState getFlywheelState() {
        sleep(50);
        return state; }

    public String getFlywheelStateString() {
        return state.toString();
    }

    public void setPhoneVision() {
        phoneServo.setPosition(visionPosition);
    }

    public void setPhoneRest() {
        phoneServo.setPosition(restPosition);
    }

    public void setPhoneDown() {
        phoneServo.setPosition(downPosition);
    }

    public void pidCalibration() {


        if(state == STATE_ACCELERATING) {
            if(timer.time() > 1000) {//Give the flywheel 1.2 seconds to power up before adjusting speed
                state = STATE_ADJUSTING;
            }
        } else {
            if (state == STATE_ADJUSTING || state == STATE_RUNNING_NEAR_TARGET) {
                int encoderReadingLeft = getEncoderLeft();
                int encoderReadingRight = getEncoderRight();
                if (lastEncoderReadingLeft == 0 && lastEncoderReadingRight == 0) {
                    //We just entered this state from the adjusting state, update encoder and time values.
                    lastEncoderReadingLeft = encoderReadingLeft;
                    lastEncoderReadingRight = encoderReadingRight;
                    timer.reset();
                } else {
                    if (timer.time() > interval) {
                        double leftDiff = encoderReadingLeft - lastEncoderReadingLeft;
                        double rightDiff = encoderReadingRight - lastEncoderReadingRight;
                        double approxRateLeft = leftDiff / timer.time();
                        double approxRateRight = rightDiff / timer.time();
                        currentErrorLeft = (approxRateLeft - targetEncoderRate) / targetEncoderRate;
                        currentErrorRight = (approxRateRight - targetEncoderRate) / targetEncoderRate;
                        if (Math.abs(currentErrorLeft) < MAX_ALLOWED_ERROR && Math.abs(currentErrorRight) < MAX_ALLOWED_ERROR) {
                            state = STATE_RUNNING_NEAR_TARGET;
                            goodTimes++;
                        } else {
                            state = STATE_ADJUSTING;
                        }
                        double errorRight = targetEncoderRate - approxRateRight;
                        double errorLeft = targetEncoderRate - approxRateLeft;
                        totalErrorRight += errorRight;
                        totalErrorLeft += errorLeft;
                        double PR = errorRight * KP;
                        double IR = totalErrorRight * KI;
                        double DR = (errorRight - prevErrorR) * KD;
                        double PL = errorLeft * KP;
                        double IL = totalErrorLeft * KI;
                        double DL = (errorLeft - prevErrorL) * KD;
                        if (PID_Modify) {
                            rightPower += PR + IR + DR;
                            leftPower += PL + IL + DL;
                            leftPower = range(leftPower);
                            rightPower = range(rightPower);
                            updatePowers();
                        }
                        count++;
                        System.out.println(count + ". Left Error: " + df.format(currentErrorLeft * 100) + " Right Error: " + df.format(currentErrorRight * 100)); //+" Time " + timer.time() + " Left " + leftDiff + " Right " + rightDiff);
                    /*opMode.telemetry.addData("Left Error(%)", df.format(currentErrorLeft*100));
                    opMode.telemetry.addData("Right Error(%)", df.format(currentErrorRight*100));
                    opMode.telemetry.addData("Left Power", flywheelLeft.getPower());
                    opMode.telemetry.addData("Right Power", flywheelRight.getPower());
                    opMode.telemetry.addData("State", state);
                    opMode.telemetry.update();*/
                        lastEncoderReadingLeft = encoderReadingLeft;
                        lastEncoderReadingRight = encoderReadingRight;
                        prevErrorR = errorRight;
                        prevErrorL = errorLeft;
                        timer.reset();

                    }
                }

            }
        }

    }

}