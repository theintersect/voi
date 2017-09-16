package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VOIColorSensor;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
import org.firstinspires.ftc.teamcode.tasks.TaskThread;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.AutoMode.AntiDefensive;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.AutoMode.Defensive;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.AutoMode.JustBeacon;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.AutoMode.ShootDefensive;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.AutoMode.TwoBall;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.DefenseMode.Beacon;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.DefenseMode.Shooting;
import static org.firstinspires.ftc.teamcode.robotutil.Team.BLUE;
import static org.firstinspires.ftc.teamcode.robotutil.Team.RED;

/**
 * Created by Howard on 12/13/16.
 * Autonomous
 */

@SuppressWarnings("ALL")
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Tests")

/**
 * The terms "up" and "back" are dependent upon team.
 * For blue team, up means driving forward and back means driving backwards.
 * For red team, up means driving backwards and back means driving forwards.
 * This is done so that "up" would always be travelling away from the start wall
 * and "back" will always be directed back towards the start wall.
 * "correct" color references the color of the team, while "wrong" references the opposing
 * alliance's color
 */

public class Autonomous extends LinearOpMode {

    // Options
    private boolean missed = false;
    private final int backID = 0x3a;
    private final int frontID = 0x3c;

    private int shootTime = 2500;

    private int betweenBeacon = 32; // far beacon distance

    // Angles
    private double shootRotation = 108; // first beacon shoot rotation near
    private double sralt3 = 90; // first beacons shoot rotation far
    private double wallAngle; // angle parallel to beacon wall
    int beaconRotation = 48;

    private double sFarRotB = 45; // shoot far rotation Blue near
    private double sFarRotB2 = 42; // shoot far rotation Blue far
    private double sFarRotR = 129; // shoot far rotation Red
    private double sCloRotB = 108; // shoot close rotation Blue
    private double sCloRotR = 80; // shoot close rotation Red

    // Powers
    private double shootPower = 0.68; // shoot first power
    private double bpPower = 0.1; // beacon pressing driveTrain power

    // Hardware
    private ColorSensor colorBack, colorFront;
    private VOIColorSensor voiColorBack, voiColorFront, voiFront, voiBack;
    // voiColor is the color sensor we use
    private Servo guide;
    private BNO055IMU adaImu;
    private VOIImu imu;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    //Misc
    private static Team team = BLUE;
    private static AutoMode autoMode = AutoMode.ThreeBall;
    private static ParkMode parkMode = ParkMode.Center;
    private static DefenseMode defenseMode = DefenseMode.Beacon;

    private static int delayTime = 0;
    private static final boolean shootFirst = true;
    private FlywheelTask flywheelTask;
    private IntakeTask intakeTask;
    private ButtonPusherTask buttonPusherTask;

    boolean farSecondBeacon = false;

    private DecimalFormat df = new DecimalFormat();

    private MecanumDriveTrain driveTrain;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime gameTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime colorTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public enum AutoMode {
        TwoBall, ThreeBall, Defensive, JustShoot, ShootDefensive, JustBeacon, AntiDefensive
    }

    public enum ParkMode {
        Corner, Center
    }

    public enum DefenseMode {
        Beacon, Shooting
    }

    @Override
    public void runOpMode() {
        initialize();
        options();
        waitForStart();
        flywheelTask.setPhoneDown();
        System.out.println("Autonomous started!");
        gameTimer.reset();
        flywheelTask.start();
        buttonPusherTask.start();
        intakeTask.start();
        while (opModeIsActive() && gameTimer.time() < delayTime);
        if (autoMode == TwoBall || autoMode == AutoMode.ThreeBall) {
            runBalls();
        } else if (autoMode == AutoMode.JustShoot) {
            runJustShoot();
        } else if (autoMode == Defensive) {
            runDefensive();
        } else if (autoMode == ShootDefensive) {
            if (defenseMode == Beacon) {
                runJustShoot();
            } else if (defenseMode == Shooting) {
                runJustShoot();
            }
        } else if (autoMode == JustBeacon) {
            runJustBeacon();
        } else if (autoMode == AntiDefensive) {
            runAntiDefensive();
        }
    }
    
    public void initialize(){

        // Initialize color sensors
        colorBack = hardwareMap.colorSensor.get("colorBack");
        colorFront = hardwareMap.colorSensor.get("colorFront");
        colorBack.setI2cAddress(I2cAddr.create8bit(backID));
        colorFront.setI2cAddress(I2cAddr.create8bit(frontID));
        voiColorBack = new VOIColorSensor(colorBack, this);
        voiColorFront = new VOIColorSensor(colorFront, this);
        voiColorBack.lightOn = false;
        voiColorFront.lightOn = false;
        // Servo for guide wheels
        guide = hardwareMap.servo.get("guide");

        // Initialize tasks, CapBallTask is for forklift initialization but is not actually used
        new CapBallTask(this);
        intakeTask = new IntakeTask(this);
        flywheelTask = new FlywheelTask(this);
        buttonPusherTask = new ButtonPusherTask(this);

        // Initialize the IMU, this will take a few seconds (majority of time)
        adaImu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new VOIImu(adaImu);

        // DriveTrain is used for majority of robot movement
        driveTrain = new MecanumDriveTrain(this);
        driveTrain.bpPower = bpPower;
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        // Bring guide position up
        guide.setPosition(ButtonPusherTask.upPosition);
        flywheelTask.setPhoneRest();

        // Tell color sensor and drive train the team color, which is important for detecting the
        // team color (correctColor) and for driving orientations (powerUp, moveUp, etc)
        voiColorBack.team = driveTrain.team = team;

        // Calculate current voltage for tasks. This determines the initial power that the motors
        // are set at.
        TaskThread.calculateVoltage(this);

        // Misc
        df.setMaximumFractionDigits(2);

    }

    public void runBalls() {
        if (autoMode == AutoMode.ThreeBall) {
            pickUp();
        } else if (autoMode == TwoBall) {
            flywheelTask.setFlywheelPow(shootPower);
            driveTrain.moveBackwardNInch(0.2, 15, 5, false, true, false);
        }
        if (shootFirst) {
            shoot();
            if (team == BLUE) {
                driveTrain.rotateToAngle(VOIImu.addAngles(wallAngle, beaconRotation));
                lineUpToWall(40);
            } else if (team == RED) {
                driveTrain.rotateToAngle(VOIImu.subtractAngles(wallAngle, beaconRotation));
                if (autoMode == TwoBall) {
                    lineUpToWall(32);
                } else {
                    lineUpToWall(36);
                }
            }
            if (team == BLUE) {
            }
        } else {
            lineUpToWall(32);
        }
        drivePushButton();
        drivePushButton2();

        if (missed) {
            checkFirst();
            knockCap();
        } else {
            if (parkMode == ParkMode.Center) {
                knockCap2();
            } else if (parkMode == ParkMode.Corner) {
                parkCorner();
            }
        }
    }

    public void runDefensive() {
        // autonomous for driving between beacons of opposing side
        buttonPusherTask.out();
        if (team == BLUE) {
            lineUpToWall(38);
        } else if (team == RED) {
            lineUpToWall(31);
        }
        drivePushButton();
        drivePushButton2();
        if (missed) {
            checkFirst();
            knockCap();
        } else {
            beaconDefense();
        }
        buttonPusherTask.in();
        sleep(1000);

    }

    public void runJustShoot() {
        // start from far corner and shoot and knock cap ball
        shootFromCorner();
        if (parkMode == ParkMode.Corner) {
            driveTrain.rotateToAngle(VOIImu.subtractAngles(wallAngle, 70));
            driveTrain.moveBackwardNInch(1, 75, 5, false, true, false);
        } else if (parkMode == ParkMode.Center) {
            driveTrain.moveBackwardNInch(0.15, 37, 5, false, true, false);
            if (autoMode == ShootDefensive) {
                driveTrain.rotateToAngle(wallAngle);
                System.out.println("Before 10? " + gameTimer.time()  );
                while (gameTimer.time() < 10000 && opModeIsActive()) ;
                driveTrain.moveUpNInch(1, 30, 10, false, true, false);
                sleep(200);
                double angle = VOIImu.subtractAngles(imu.getAngle(), wallAngle);
                int distance = 30;
                if (team == RED) {
                    distance += 8;
                }
                driveTrain.rotateToAngle(wallAngle);
                driveTrain.moveLeftNInch(1, angle*2/3, 5, false, true, true);
                driveTrain.moveUpNInch(1, distance, 10, false, true, false);
                driveTrain.holdPosition();
                telemetry.addData("Time", gameTimer.time()*1.0/1000);
                System.out.println(gameTimer.time() * 1.0/1000);
                telemetry.update();
                while (gameTimer.time() < 30000 & opModeIsActive()) ;
            }
        }
    }

    public void runBlockShooting() {
        shootFromCorner();
        double firstRotation = VOIImu.subtractAngles(wallAngle, 90);
        driveTrain.rotateToAngle(wallAngle);
        while (gameTimer.time() < 10000 && opModeIsActive());
        driveTrain.moveForwardNInch(1, 50, 5, false, true, false);
        sleep(1000);
        driveTrain.rotateToAngle(wallAngle);
        driveTrain.moveUpNInch(1, 24, 5, false, true, false);
        sleep(1000);
        driveTrain.moveBackwardNInch(1, 36, 10, false, true, false);
    }

    public void shoot() {
        flywheelTask.setPriority(Thread.MAX_PRIORITY);
        flywheelTask.setPhoneDown();
        System.out.println("Shoot");
        sleep(2500);
        //flywheelTask.PID_Modify = false;
        intakeTask.setPower(1);
        timer.reset();
        while (timer.time() < shootTime && opModeIsActive());
        flywheelTask.setFlywheelPow(0);
        intakeTask.setPower(0);
        flywheelTask.setPriority(Thread.MIN_PRIORITY);
    }

    public void pickUp() {
        /**
         * This method begins with the robot facing away from the corner vortex. The ball should be
         * right under the sweeper (but only touching the alliance robot). The goal is to pick up
         * the third ball, shoot all three balls, and end facing towards the white line of the
         * first beacon.
         * 
         * Steps:
         *  1. Run sweeper for 1.25 seconds.
         *  2. Strafe away from wall towards center vortex.
         *  3. Rotate so that flywheels are pointed towards center vortex.
         *  4. Run flywheels and sweeper to shoot balls.
         *  5. Rotate so that appropriate end is oriented towards white line of the first beacon.
         */
        
        // 1.
        flywheelTask.setFlywheelPow(shootPower);
        int sweepTime = 1000;
        powerSweeper(1, sweepTime);
        sleep(sweepTime);
        // increase shootTime to account for third ball
        shootTime = 4000;

        // 2.
        if (shootFirst) {
            // 3.
            if (team == BLUE) {
                driveTrain.teamStrafeRightNInch(1, 15, 10, false, true, true);
                driveTrain.rotateToAngle(wallAngle + 180, 0.25, 1.5, 6);
            } else if (team == RED) {
                driveTrain.moveLeftNInch(1, 18, 10, false, true, true);
                driveTrain.rotateToAngle(wallAngle, 0.25, 1.5, 6);
            }

            // 4.
        } else {
            driveTrain.moveRightNInch(1, 5, 5, false, true, true);
            driveTrain.rotateToAngle(wallAngle);
        }


    }

    public void lineUpToWall(double distance) {
        /**
         * This method begins with the robot roughly pointing at the white line of the beacon.
         * The goal of this method is to end with the robot aligned with the wall, allowing it to
         * move along it while detecting and pushing beacons.
         *
         * Steps:
         *   1. Drive a set distance that should end end up with robot approximately at white line.
         *   2. Rotate so that robot is parallel with wall (to angle of initialization).
         *   3. Strafe towards the wall with considerable power to ensure lining up correctly.
         */

        telemetry.addData("lineUpToWall", "");
        telemetry.update();

        buttonPusherTask.out();
        guide.setPosition(ButtonPusherTask.downPosition);
        int wlTimeout = 0; // timeout for white line detection (used for missing line)
        double fastDistance = distance - 5.5;
        double fastPower = 0.8;
        driveTrain.moveUpNInch(0.35, 0.5, 10, false, false, false);
        driveTrain.moveUpNInch(fastPower, fastDistance, 10, false, false, false);
        driveTrain.moveUpNInch(0.35, 5, 2, false, true, false);
        sleep(100);
        driveTrain.rotateToAngle(wallAngle);
        sleep(50);
        if (autoMode == Defensive) {
            sleep(250);
        }
        if (team == BLUE) {
            driveTrain.moveRightNInch(1, 45, 4, true, true, true);
        } else if (team == RED) {
            driveTrain.moveRightNInch(1, 45, 4, true, true, true);
        }
        correctionStrafe();
    }

    public void drivePushButton() {
        
        /* This method assumes that the robot is already lined up to the wall and is
         * behind the first beacon. We do not want to spend time checking if the robot is in
         * front of the beacon, as that is unnecessary (and would be a fix in the lineUpToWall
         * method call.) The goal of the drivePushButton() method is to push the first beacon.
         * 
         * Steps:
         * 1. Activate the button pusher if the correct color is detected immediately.
         * 2. If the color sensor reads no color (or the wrong color), drive forward slowly until
         *    the correct color is detected and activate the button pusher
         * 3. If the top color sensor does not detect the correct color, then it assumes
         *    that the beacon turned the wrong color when ramming into the wall. Because of the 5s
         *    delay, the robot moves on to the second beacon and returns to the first beacon later.
         *    The "betweenBeacon" distance (how much the robot moves before starting to detect color
         *    again) is decreased to account for the extra distance moved in the timeout.
         */

        telemetry.addData("drivePushButton", "");
        telemetry.update();

        int timeOut = 2000;
        timer.reset();
        colorTimer.reset();
        if (voiBack.correctColor() &&  !voiFront.correctColor()) {
            System.out.println("First Condition: ");
            System.out.println("Back Color: " + voiBack.getColor());
            System.out.println("Front Color: " + voiFront.getColor());
            driveTrain.moveBackNInch(0.15, 1, 2, false, true, true);
            pushButton();
            return;
        } else if (voiFront.correctColor() && !voiBack.correctColor()) {
            System.out.println("Second Condition:");
            System.out.println("Back Color: " + voiBack.getColor());
            System.out.println("Front Color: " + voiFront.getColor());
            // 1.
            driveTrain.moveUpNInch(0.1, 1, 2, false, true, true);
            pushButton();
            // If the correct color is detected immediately, then we assume that the we pressed
            // the side farther from the second beacon. We increase the "betweenBeacon" distance to
            // account for that.
            betweenBeacon += 3;
            return;
        } else {
            // 2.
            driveTrain.crawlUp();
            timer.reset();
            boolean timeAdded = false;

            while (timer.time() < timeOut && opModeIsActive()) {
                if (voiBack.correctColor() && !voiFront.correctColor()) {
                    System.out.println("Back detect, front not, while drive");
                    driveTrain.moveBackNInch(0.15, 1, 2, false, true, true);
                    pushButton();
                    return;
                }
                if (voiFront.wrongColor() && !timeAdded) {
                    // If the opposite color is detected, add 1 second to the timeout in case the
                    // robot starts further back than expected.
                    timeOut += 1000;
                    timeAdded = true;
                }
                if (colorTimer.time() > 50) {
                    System.out.println("Driving: ");
                    System.out.println("Back Color: " + voiBack.getColor());
                    System.out.println("Front Color: " + voiFront.getColor());
                    colorTimer.reset();
                }
                // Turning because of known rotation after stopping. Temporary fix.
                if (voiFront.correctColor()) {
                    pushButton();

                    return;
                }

            }
        }
        
        // 3.
        missed = true;
    }
    
    public void drivePushButton2() {
        /*  This method starts with the robot after it has pressed the first beacon (or given up.)
         *  The goal is to press the correct color on the second beacon.
         *
         *  Steps:
         *      1. Drive "betweenBeacon" distance before beginning to detect for the second beacon.
         *         This driving is done at a faster velocity to increase speed.
         *      2. Drive slower and look for beacon.
         *      3. When the correct color is detected, activate button pusher
         *
         */
        telemetry.addData("drivePushButton2", "");
        telemetry.update();
        int timeo = 8000;
        double buffer = 15;
        // 1.
        if (!missed) {
            driveTrain.moveUpNInch(0.4, betweenBeacon - buffer, 10, false, false, true);
        }

        driveTrain.moveUpNInch(0.10, buffer, 3, false, false, true);


        // 2.
        driveTrain.crawlUp();
        boolean detectColor = false;
        timer.reset();
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeout.reset();
        colorTimer.reset();
        while (!detectColor && opModeIsActive() && timeout.time() < timeo) {
                // Boolean "far" determines the angle the robot turns to hit the cap ball.
            if (voiFront.wrongColor() && !farSecondBeacon) {
                farSecondBeacon = true;
                timeo += 1000;
            }

            detectColor = voiFront.correctColor();

            if (voiBack.correctColor() && !voiFront.correctColor()) {
                System.out.println("Back detect, front not, while drive");
                driveTrain.moveBackNInch(0.15, 1, 2, false, true, true);
                pushButton();
                return;
            }
            if (colorTimer.time() > 50) {
                System.out.println("Driving 2: ");
                System.out.println("Back Color: " + voiBack.getColor());
                System.out.println("Front Color: " + voiFront.getColor());
                colorTimer.reset();
            }
        }


        // 3.
        pushButton();
        if (farSecondBeacon) {
            sFarRotB = sFarRotB2;
        }
        if (!missed) {
            // if all goes well, withdraw button pusher and lift guide wheels
            guide.setPosition(ButtonPusherTask.upPosition);
        }
    }

    public void knockCap() {
        /**
         * This method starts right after the robot pushes the first beacon (after coming back from
         * the second beacon). The goal is to knock the cap ball off the center vortex and park.
         *
         * Steps:
         * 1. Strafe left from the wall, as you cannot rotate when right next to the wall.
         * 2. Rotate so that back of robot is oriented towards center vortex.
         * 3. Hit cap ball
         */
        buttonPusherTask.in();
        guide.setPosition(ButtonPusherTask.upPosition);
        telemetry.addData("knockCap", "");
        telemetry.update();

        if (parkMode == ParkMode.Center) {
            // 1.
            driveTrain.moveLeftNInch(1, 6, 5, false, false, true);
            driveTrain.stopAll();
            guide.setPosition(ButtonPusherTask.upPosition);
            // 2.
            if (team == BLUE) {
                driveTrain.rotateToAngle(wallAngle + sCloRotB);
            } else if (team == RED) {
                driveTrain.rotateToAngle(wallAngle + sCloRotR);
            }

            //3.
            driveTrain.moveBackwardNInch(1, 50, 10, true, true, false);
            driveTrain.rotateDegrees(-270, 1, false);
            driveTrain.moveBackwardNInch(1, 18, 10, true, true, false);
        } else if (parkMode == ParkMode.Corner) {
            driveTrain.moveLeftNInch(1, 3, 1, false, true, false);
            driveTrain.moveBackNInch(1, 30, 10, true, true, false);
        }
    }

    public void knockCap2() {
        /**
         * This method starts right after the robot has pressed the second beacon. The goal is to
         * point towards the center vortex and knock the cap ball off and park on the center vortex.
         *
         * Steps:
         * 1. Strafe away from the wall
         * 2. Rotate so that back is faced towards center vortex.
         * 3. Move backwards and knock the cap ball off.
         */

        telemetry.addData("knockCap2", "");
        telemetry.update();
        buttonPusherTask.in();
        // lift up guide wheels
        guide.setPosition(ButtonPusherTask.upPosition);

        // 1.
        driveTrain.moveLeftNInch(1, 3, 10, false, true, true);

        // 2.
        if (team == BLUE) {
            driveTrain.rotateToAngle(wallAngle + sFarRotB);
        } else if (team == RED) {
            sFarRotR += 180;
            driveTrain.rotateToAngle(wallAngle + sFarRotR);
        }

        // 3.
        driveTrain.moveBackNInch(0.5, 55, 10, false, true, false);
        driveTrain.stopAll();

    }
    
    public void parkCorner() {
        /**
         * This method starts right after the robot has pressed the second beacon. The goal is to
         * park onto the corner vortex (only run when the alliance robot can knock off cap ball)
         *
         * Steps:
         * 1. Strafe away from the wall
         * 2. Rotate so that back is faced towards center vortex.
         * 3. Move backwards and knock the cap ball off.
         */
        buttonPusherTask.in();
        driveTrain.moveLeftNInch(1, 25, 4, false, true, true);
        sleep(200);
        driveTrain.rotateToAngle(wallAngle);
        guide.setPosition(ButtonPusherTask.upPosition);
        double tiltRoll = imu.getRoll() + 6;
        driveTrain.moveBackNInch(1, 85, 10, true, true, true);

        /*if (team == BLUE) {
            driveTrain.powerAllMotors(-1);
            double target = backRight.getCurrentPosition() + 95 * MecanumDriveTrain.TICKS_PER_INCH_FORWARD;
            while (opModeIsActive() && backRight.getCurrentPosition() < target) {
                if (imu.getRoll() > tiltRoll) {
                    sleep(1000);
                    return;
                }
            }
        } else if (team == RED) {
            driveTrain.powerAllMotors(1);
            double target = backRight.getCurrentPosition() - 95 * MecanumDriveTrain.TICKS_PER_INCH_FORWARD;
            while (opModeIsActive() && backRight.getCurrentPosition() > target) {
                if (imu.getRoll() > tiltRoll) {
                    sleep(1000);
                    return;
                }
            }
        }*/
    }

    public void checkFirst() {
        /**
         * This method starts right after pressing the second beacon (and skipping the first).
         * The goal is to return to the first beacon and convert it to the alliance color.
         *
         * Steps:
         *  1. Move back a certain distance (to ensure that robot is behind the second beacon)
         *  2. Move back slowly while looking for the correct color.
         *  3. Activate button pusher when correct color detected.
         */
        telemetry.addData("checkFirst", "");

        telemetry.update();
        // 1.
        driveTrain.moveBackNInch(0.3, betweenBeacon - 5, 10, false, false, true);
        driveTrain.moveBackNInch(0.15, 5, 10, false, true, true);
        correctionStrafe();

        // 2.
        driveTrain.crawlBack();
        boolean isWrong = false;
        boolean rammedWrong = false;
        while (opModeIsActive() && !voiBack.correctColor() && !rammedWrong) {
            if (voiBack.wrongColor() && !isWrong) {
                isWrong = true;
                timer.reset();
            }
            if (isWrong && timer.time() > 750) {
                // If, after detecting the wrong color for some time, the correct color is still not
                // detected, then we can infer that we activated the button pusher incorrectly upon
                // ramming.
                rammedWrong = true;
                driveTrain.stopAll();
            }
        }
        if (rammedWrong) {
            // go back and look for the wrong color to press
            correctionStrafe();
            driveTrain.crawlUp();
            while (opModeIsActive() && !voiBack.wrongColor());
            pushButton();
        } else {
            pushButton();
        }
        if (!isWrong) {
            shootRotation = sralt3;
        }

    }

    public void coolDown() {
        intakeTask.setPower(0);
        flywheelTask.setFlywheelPow(0);
    }

    public void correctionStrafe() {
        correctionStrafe(0.5);
    }

    public void correctionStrafe(double seconds) {
        // Strafing against wall to ensure alignment. Same direction regardless of color because
        // button pusher is always on the right side.
        driveTrain.stopAll();
        if (team == RED) {
            driveTrain.rotateToAngle(wallAngle, 0.5, 4, 0.75);
        }
        driveTrain.moveRightNInch(1, 5, seconds, false, true, false);
    }

    public void pushButton() {
        // Stop driving and push the button. Wait a little more than the required push time to allow
        // button to partially retract.
        correctionStrafe();
        buttonPusherTask.push();
        sleep(buttonPusherTask.pushTime + 200);
    }

    public void powerSweeper(double power, int time) {
        intakeTask.power = power;
        intakeTask.sweepTime = time;
    }

    public void beaconDefense() {
        buttonPusherTask.in();
        guide.setPosition(ButtonPusherTask.upPosition);
        driveTrain.moveLeftNInch(1, 54, 10, false, true, true);
        driveTrain.rotateToAngle(wallAngle);
        double distance = 27;
        if (farSecondBeacon) {
            distance -= 4;
        }
        if (team == BLUE) {
            distance -= 12;
        }
        driveTrain.moveUpNInch(0.5,distance, 5, false, true, false);
        driveTrain.holdPosition();
        telemetry.addData("Time", gameTimer.time()*1.0/1000);
        System.out.println(gameTimer.time() * 1.0/1000);
        telemetry.update();
        while (gameTimer.time() < 27000 && opModeIsActive());
        driveTrain.moveBackNInch(0.5, 60, 3, false, true, false);
    }

    public void backToFirst() {
        driveTrain.moveBackNInch(0.4, betweenBeacon - 8, 10, false, false, true);
        driveTrain.moveBackNInch(0.11, 8, 5, false, true, true);
        driveTrain.crawlBack();
        while (opModeIsActive() && !voiFront.correctColor());
        driveTrain.stopAll();
    }

    public void options() {
        boolean confirmed = false;
        boolean rTrigger = false;
        boolean lTrigger = false;
        while(!confirmed && !isStopRequested()){

            // select team
            if (gamepad1.b){
                team = RED;
            } else if (gamepad1.x){
                team = BLUE;
            }

            // select auto mode
            if (gamepad1.dpad_left) {
                autoMode = TwoBall;
            } else if (gamepad1.dpad_up) {
                autoMode = AutoMode.ThreeBall;
            } else if (gamepad1.dpad_right) {
                autoMode = Defensive;
            } else if (gamepad1.dpad_down) {
                autoMode = AutoMode.JustShoot;
            } else if (gamepad1.left_stick_x < -0.85) {
                autoMode = ShootDefensive;
            } else if (gamepad1.left_stick_y < -0.85) {
                autoMode = JustBeacon;
            } else if (gamepad1.left_stick_x > 0.85) {
                autoMode = AntiDefensive;
            }

            // select park mode
            if (gamepad1.right_bumper) {
                parkMode = ParkMode.Center;
                defenseMode = Beacon;
            } else if (gamepad1.left_bumper) {
                parkMode = ParkMode.Corner;
                defenseMode = Shooting;
            }

            if (gamepad1.right_trigger > 0.15 && !rTrigger) {
                rTrigger = true;
                delayTime += 500;
            }
            if (gamepad1.left_trigger > 0.15 && !lTrigger) {
                lTrigger = true;
                delayTime -= 500;
            }
            if (gamepad1.right_trigger < 0.15) {
                rTrigger = false;
            }
            if (gamepad1.left_trigger < 0.15) {
                lTrigger = false;
            }
            telemetry.addData("Team", team == RED ? "Red" : "Blue");
            telemetry.addData("Mode", autoMode);
            switch (autoMode) {
                case JustShoot:
                    telemetry.addData("Park", parkMode);
                    break;
                case ThreeBall:
                    telemetry.addData("Park", parkMode);
                    break;
                case TwoBall:
                    telemetry.addData("Park", parkMode);
                    break;
                case Defensive:
                    telemetry.addData("Defense Mode", defenseMode);
                    break;
                case ShootDefensive:
                    telemetry.addData("Defense Mode", defenseMode);
                    break;
                case JustBeacon:
                    telemetry.addData("Park", parkMode);
            }
            telemetry.addData("Delay", (double)delayTime/1000);

            if ((gamepad1.left_stick_button && gamepad1.right_stick_button) || isStarted()){
                telemetry.addData("Confirmed!", "");
                if (team == BLUE) {
                    voiFront = voiColorFront;
                    voiBack = voiColorBack;
                } else if (team == RED){
                    voiFront = voiColorBack;
                    voiBack = voiColorFront;
                }
                voiFront.team = driveTrain.team = voiBack.team = team;
                intakeTask.setTeam(team);
                confirmed = true;

                switch (autoMode) {
                    case JustShoot:
                        betweenBeacon += 3;
                        bpPower += 0.05;
                        wallAngle = imu.getAngle();
                        break;
                    case TwoBall:
                        shootPower = 0.66;
                        if (team == RED) {
                            wallAngle = imu.getAngle();
                        } else if (team == BLUE) {
                            wallAngle = VOIImu.addAngles(wallAngle, 180);
                        }
                        break;
                    case ThreeBall:
                        // same for both sides
                        wallAngle = VOIImu.addAngles(imu.getAngle(), 90);
                        break;
                    case Defensive:
                        wallAngle = imu.getAngle();
                        shootTime = 1500;
                        buttonPusherTask.setPriority(Thread.MAX_PRIORITY);
                        break;
                    case ShootDefensive:
                        shootPower = 0.72;
                        wallAngle = imu.getAngle();
                        shootTime = 1500;
                        break;
                    case JustBeacon:
                        wallAngle = imu.getAngle();
                        buttonPusherTask.setPriority(Thread.MAX_PRIORITY);
                        break;

                }
            }
            telemetry.update();

        }
    }

    public void beaconTest() {
        for (int i = 0; i < 10; i ++) {
            if (!opModeIsActive()) {
                break;
            }
            buttonPusherTask.pushTime = 400;
            betweenBeacon = 33;
            guide.setPosition(ButtonPusherTask.downPosition);
            buttonPusherTask.out();
            sleep(1000);
            gameTimer.reset();
            drivePushButton();
            drivePushButton2();
            System.out.println(gameTimer.time());
            buttonPusherTask.in();
            sleep(1000);
            driveTrain.moveBackNInch(0.25, 55, 10, false, true, true);
            flywheelTask.running = false;
        }
    }

    void shootFromCorner() {
        driveTrain.moveBackwardNInch(0.3, 30, 10, false, true, false);
        flywheelTask.setFlywheelPow(shootPower);
        shoot();
    }

    void runJustBeacon() {
        buttonPusherTask.out();
        if (team == BLUE) {
            lineUpToWall(36);
        } else if (team == RED) {
            lineUpToWall(31);
        }
        drivePushButton();
        flywheelTask.setFlywheelPow(shootPower);
        drivePushButton2();
        if (missed) {
            checkFirst();
            knockCap();
        } else {
            if (parkMode == ParkMode.Center) {
                knockCap2();
            } else if (parkMode == ParkMode.Corner) {
                //parkCorner
                buttonPusherTask.in();
                driveTrain.moveLeftNInch(1, 7, 4, false, true, true);
                double angle = VOIImu.addAngles(wallAngle, 43.5);
                driveTrain.rotateToAngle(angle);
                driveTrain.moveBackwardNInch(0.3, 25, 5, false, true, false);
                shoot();
                driveTrain.moveBackwardNInch(1, 30, 5, true, true, false);
            }
        }
        buttonPusherTask.in();
        sleep(1000);
    }

    void runAntiDefensive() {
        shootFromCorner();
        driveTrain.moveBackwardNInch(0.25, 70, 10, false, true, false);
        driveTrain.holdPosition();
        while (opModeIsActive() && gameTimer.time() < 25000);
        driveTrain.moveForwardNInch(0.15, 30, 5, false, true, false);
    }

}