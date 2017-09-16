package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.vision.LinearOpModeVision;
import org.lasarobotics.vision.detection.objects.Contour;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.opmodes.VisionShootingTest.VORTEX_TARGET;
import static org.firstinspires.ftc.teamcode.opmodes.VisionShootingTest.VisionMode.PARTICLES;
import static org.firstinspires.ftc.teamcode.opmodes.VisionShootingTest.VisionMode.VORTEX;
import static org.firstinspires.ftc.teamcode.robotutil.Team.BLUE;
import static org.firstinspires.ftc.teamcode.robotutil.Team.RED;

/**
 * Created by Howard on 12/13/16.
 * Autonomous
 */

@SuppressWarnings("ALL")
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoWithVision", group = "Tests")

/**
 * The terms "up" and "back" are dependent upon team.
 * For blue team, up means driving forward and back means driving backwards.
 * For red team, up means driving backwards and back means driving forwards.
 * This is done so that "up" would always be travelling away from the start wall
 * and "back" will always be directed back towards the start wall.
 * "correct" color references the color of the team, while "wrong" references the opposing
 * alliance's color
 */
public class AutoWithVision extends LinearOpModeVision {

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

    private double sFarRotB = 42; // shoot far rotation Blue near
    private double sFarRotB2 = 42; // shoot far rotation Blue far
    private double sFarRotR = 129; // shoot far rotation Red
    private double sCloRotB = 108; // shoot close rotation Blue
    private double sCloRotR = 80; // shoot close rotation Red

    // Powers
    private double shootPower = 0.72; // shoot first power
    private double bpPower = 0.1; // beacon pressing driveTrain power

    // Hardware
    private ColorSensor colorBack, colorFront;
    private VOIColorSensor voiColorBack, voiColorFront, voiFront, voiBack;
    // voiColor is the color sensor we use
    private Servo guide;
    private BNO055IMU adaImu;
    private VOIImu imu;

    VisionShootingTest.VisionMode prevVisMode = PARTICLES;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    boolean visionActive = false;

    //Misc
    private static Team team = Team.BLUE;
    private static AutoMode autoMode = AutoMode.ThreeBall;
    private static ParkMode parkMode = ParkMode.Center;
    private static int delayTime = 0;
    private static final boolean shootFirst = true;
    private FlywheelTask flywheelTask;
    private IntakeTask intakeTask;
    private ButtonPusherTask buttonPusherTask;

    private DecimalFormat df = new DecimalFormat();

    private MecanumDriveTrain driveTrain;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime gameTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime timer2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime colorTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    
    // VISION STUFF ----------------------
    Mat mHsvMat;
    Mat mRgbaMat;
    Mat red1;
    Mat red2;
    Mat colorMask;
    Mat grayMask;
    Mat lowValMask;
    Mat lowSatMask;
    Mat blurred;
    Mat circles;
    MatOfInt convexHull;
    MatOfPoint2f temp2fMat;
    MatOfPoint2f polyApprox;
    MatOfInt4 convexityDefects;
    List<MatOfPoint> initialContourList;
    List<MatOfPoint> potentialContours;
    List<MatOfPoint> grayContours;
    List<Contour> resultContours;
    List<MatOfPoint> passedFirstCheck;

    Servo phoneServo;

    boolean detectedVortex = false;
    final double shootDistance = 36;

    int centX = -1;
    int centY = -1;
    double zeroAngle = 0;

    boolean explore = true;

    static final int TARGET_VORTEX_HEIGHT = 650;
    static final int VORTEX_THRESHOLD = 8000;
    static final int IMAGE_HEIGHT = 600;       //ZTE Camera picture size
    static final int IMAGE_WIDTH = 800;
    static final double MAX_VORTEX_AREA_RATIO = 0.6;

    static final double visionPosition = 0.2, downPostion = 0, restPosition = 0.68;

    static final int ACCEPTABLE_ERROR = 50;
    static final int PARTICLE_TARGET = 320;
    static final double MIN_PARTICLE_AREA_RATIO = 0.6;
    static final double PARTICLE_MIN_THRESHOLD = 500;
    static final double PARTICLE_MAX_THRESHOLD = 20000;
    double correctPower = 0.03;

    double[] blackArray = new double[] {0, 0, 0, 0};
    Scalar blackScalar = new Scalar(0, 0, 0);


    double aimVortexAngle;
    // this is for vision
    static int minVortexBlueH = 96;
    static int maxVortexBlueH = 120;
    static int minVortexBlueSat = 50;
    static int maxVortexBlueVal = 200;

    static int minBlueH = 80;
    static int maxBlueH = 110;
    static int minRedH = 160;
    static int maxRedH = 200;
    static int minBlueSat = 125;
    static int minRedSat = 50;
    static int minBlueVal = 75;
    static int minRedVal = 50;

    int cameraNumber = 0;
    boolean detectedTarget = false;
    ElapsedTime rejectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int rejectTime = 500;
    ElapsedTime pauseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    VisionShootingTest.VisionMode visMode = PARTICLES;

    boolean ballDetected = false;
    boolean scanWall = false;

    // location stuff
    double robotX = 35;
    double robotY = 15;
    double robotAngle = 0;
    double vortexX = 80;
    double vortexY = 64;

    int vortexBottom = -1;
    private enum AutoMode {
        TwoBall, ThreeBall, VisionBeacon, JustShoot
    }

    private enum ParkMode {
        Corner, Center
    }

    @Override
    public void runOpMode() {
        initialize();
        initCamera(cameraNumber);   //Start OpenCV
        initVision();
        changeVisMode(PARTICLES);
        options();
        if (autoMode == AutoMode.VisionBeacon) {
            optionsVision();
            setInitialPosition();
        }

        waitForStart();
        aimVortexAngle = imu.getAngle();
        System.out.println("Autonomous started!");
        gameTimer.reset();
        flywheelTask.start();
        buttonPusherTask.start();
        intakeTask.start();
        flywheelTask.setPhoneDown();
        while (opModeIsActive() && gameTimer.time() < delayTime)
            idle();
        if (autoMode == AutoMode.TwoBall || autoMode == AutoMode.ThreeBall) {
            runBalls();
        } else if (autoMode == AutoMode.JustShoot) {
            runVisionShoot();
        } else if (autoMode == AutoMode.VisionBeacon) {
            runBeaconWithVision();
        }
        stopCamera();
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



        // Calculate current voltage for tasks. This determines the initial power that the motors
        // are set at.
        TaskThread.calculateVoltage(this);

        // Misc
        df.setMaximumFractionDigits(2);

        // VISION STUFF ----------
        phoneServo = hardwareMap.servo.get("phoneServo");
        phoneServo.setPosition(restPosition);
        zeroAngle = driveTrain.getAngle();
        if (team == RED) {
            double temp = vortexX;
            //noinspection SuspiciousNameCombination
            vortexX = vortexY;
            vortexY = temp;
            double temp2 = robotX;
            robotX = robotY;
            robotY = temp2;
        }

    }

    public void runBalls() {
        if (autoMode == AutoMode.ThreeBall) {
            pickUp();
        } else if (autoMode == AutoMode.TwoBall) {
            flywheelTask.setFlywheelPow(shootPower);
            driveTrain.moveBackwardNInch(0.2, 15, 5, false, true, false);
        }
        if (shootFirst) {
            shoot();
            if (team == Team.BLUE) {
                driveTrain.rotateToAngle(VOIImu.addAngles(wallAngle, beaconRotation));
            } else if (team == Team.RED) {
                driveTrain.rotateToAngle(VOIImu.subtractAngles(wallAngle, beaconRotation));
            }
            lineUpToWall(40);
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

    public void runVisionBeacon() {
        // autonomous for driving between beacons of opposing side
        buttonPusherTask.out();
        lineUpToWall(40);
        drivePushButton();
        drivePushButton2();
        if (missed) {
            checkFirst();
            knockCap();
        } else {
            //defense();
        }
        buttonPusherTask.in();
        sleep(1000);

    }

    public void runJustShoot() {
        // start from far corner and shoot and knock cap ball
        driveTrain.moveBackwardNInch(0.3, 30, 10, false, true, false);
        flywheelTask.setFlywheelPow(shootPower);
        shoot();
        changeVisMode(PARTICLES);
        while(opModeIsActive()) {
            // too much to right is too big y, is negative power
            if (cameraNumber == 1) {
                correctPower = -0.03;
            } else {
                correctPower = 0.03;
            }
            if (rotateAim()) {
                if (foundTarget())
                    return;
            }
        }
    }

    public void runBeaconWithVision() {
        if (team == BLUE)
            lineUpToWall(40);
        else {
            lineUpToWall(35);
        }
        flywheelTask.setPhoneVision();
        drivePushButton();
        drivePushButton2();
        if (missed) {
            checkFirst();
            knockCap();
        } else {
            buttonPusherTask.in();
            guide.setPosition(ButtonPusherTask.upPosition);
            sleep(500);
            pickUpBallAndShoot();
        }
    }

    public void shoot() {
        flywheelTask.setPhoneDown();
        System.out.println("Shoot");
        sleep(1500);
        timer.reset();
        intakeTask.setPower(1);
        timer.reset();
        while (timer.time() < shootTime && opModeIsActive());
        flywheelTask.setFlywheelPow(0);
        intakeTask.setPower(0);
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
        flywheelTask.setFlywheelPow(shootPower + 0.015);
        int sweepTime = 1000;
        powerSweeper(1, sweepTime);
        sleep(sweepTime);
        //powerSweeper(-1, 650);
        //powerSweeper(-1, 250);
        // increase shootTime to account for third ball
        shootTime = 4000;

        // 2.
        if (shootFirst) {
            driveTrain.teamStrafeRightNInch(1, 15, 10, false, true, true);
            // 3.
            if (team == Team.BLUE) {
                driveTrain.rotateToAngle(wallAngle + 180, 0.25, 1.5, 6);
            } else if (team == Team.RED) {
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
        double fastDistance = distance - 0.5;
        double fastPower = 0.8;
        driveTrain.moveUpNInch(0.35, 0.5, 10, false, false, false);
        driveTrain.moveUpNInch(fastPower, fastDistance, 10, false, true, false);
        sleep(50);
        driveTrain.rotateToAngle(wallAngle);
        sleep(50);
        if (team == Team.BLUE) {
            driveTrain.moveRightNInch(1, 45, 3, true, true, false);
        } else if (team == Team.RED) {
            driveTrain.moveRightNInch(1, 45, 3, true, true, false);
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

        int timeo = 3000;
        if (missed) {
            timeo = 8000;
        }
        double buffer = 15;
        // 1.
        if (!missed) {
            driveTrain.moveUpNInch(0.4, betweenBeacon - buffer, 10, false, false, true);
        }

        driveTrain.moveUpNInch(0.10, buffer, 3, false, false, true);
        correctionStrafe();

        // 2.
        driveTrain.crawlUp();
        boolean detectColor = false;
        timer.reset();
        ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeout.reset();
        boolean far = false;
        colorTimer.reset();
        // look for second beacon
        while (!detectColor && opModeIsActive() && timeout.time() < timeo) {
            if (timer.time() > 30) {
                // Boolean "far" determines the angle the robot turns to hit the cap ball.
                if (voiFront.wrongColor() && !far) {
                    far = true;
                    timeo += 1000;
                }

                detectColor = voiFront.correctColor();
                timer.reset();
            }
            if (colorTimer.time() > 50) {
                System.out.println("Driving 2: ");
                System.out.println("Back Color: " + voiBack.getColor());
                System.out.println("Front Color: " + voiFront.getColor());
                colorTimer.reset();
            }
        }
        // if missed second beacon
        if (timeout.time() > timeo) {
            correctionStrafe(2);
            while (!detectColor && opModeIsActive()) {
                driveTrain.crawlBack();
                // Boolean "far" determines the angle the robot turns to hit the cap ball.
                if (voiBack.wrongColor() && !far) {
                    far = true;
                }
                detectColor = voiBack.correctColor();
                if (colorTimer.time() > 50) {
                    System.out.println("Driving 2 Back: ");
                    System.out.println("Back Color: " + voiBack.getColor());
                    System.out.println("Front Color: " + voiFront.getColor());
                    colorTimer.reset();
                }
            }
        }
        // 3.
        pushButton();
        if (far) {
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
        telemetry.addData("knockCap", "");
        telemetry.update();

        // 1.
        driveTrain.moveLeftNInch(1, 6, 5, false, false, true);
        driveTrain.stopAll();
        guide.setPosition(ButtonPusherTask.upPosition);
        // 2.
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sCloRotB);
        } else if (team == Team.RED){
            driveTrain.rotateToAngle(wallAngle + sCloRotR);
        }

        //3.
        driveTrain.moveBackwardNInch(1, 50, 10, true, true, false);
        driveTrain.rotateDegrees(-270, 1, false);
        driveTrain.moveBackwardNInch(1, 18, 10, true, true, false);
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
        if (team == Team.BLUE) {
            driveTrain.rotateToAngle(wallAngle + sFarRotB);
        } else if (team == Team.RED) {
            sFarRotR += 180;
            driveTrain.rotateToAngle(wallAngle + sFarRotR);
        }

        // 3.
        driveTrain.moveBackNInch(0.5, 50, 10, false, false, false);
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
        driveTrain.moveLeftNInch(1, 12, 5, false, true, true);
        sleep(200);
        driveTrain.rotateToAngle(wallAngle);
        guide.setPosition(ButtonPusherTask.upPosition);
        double tiltRoll = imu.getRoll() + 10;
        driveTrain.moveBackNInch(1, 90, 10, false, true, false);
        if (team == Team.BLUE) {
            double target = backRight.getCurrentPosition() + 95 * MecanumDriveTrain.TICKS_PER_INCH_FORWARD;
            while (opModeIsActive() && backRight.getCurrentPosition() < target) {
                if (imu.getRoll() > tiltRoll) {
                    sleep(1000);
                    return;
                }
            }
        } else if (team == Team.RED) {
            double target = backRight.getCurrentPosition() - 95 * MecanumDriveTrain.TICKS_PER_INCH_FORWARD;
            while (opModeIsActive() && backRight.getCurrentPosition() > target) {
                if (imu.getRoll() > tiltRoll) {
                    sleep(1000);
                    return;
                }
            }
        }
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

    public void correctionStrafe() {
        correctionStrafe(0.5);
    }

    public void correctionStrafe(double seconds) {
        // Strafing against wall to ensure alignment. Same direction regardless of color because
        // button pusher is always on the right side.
        driveTrain.stopAll();
        if (team == Team.RED) {
            driveTrain.rotateToAngle(wallAngle, 0.25, 2, 0.75);
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

    public void defense() {
        driveTrain.moveLeftNInch(1, 43, 10, false, true, true);
        driveTrain.rotateToAngle(wallAngle);
        driveTrain.moveUpNInch(0.5,20, 5, false, true, false);
        driveTrain.holdPosition();
        telemetry.addData("Time", gameTimer.time()*1.0/1000);
        System.out.println(gameTimer.time() * 1.0/1000);
        telemetry.update();
        while (gameTimer.time() < 30000 && opModeIsActive());

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
                team = Team.RED;
            } else if (gamepad1.x){
                team = Team.BLUE;
            }

            // select auto mode
            if (gamepad1.dpad_left) {
                autoMode = AutoMode.TwoBall;
            } else if (gamepad1.dpad_up) {
                autoMode = AutoMode.ThreeBall;
            } else if (gamepad1.dpad_right) {
                autoMode = AutoMode.VisionBeacon;
            } else if (gamepad1.dpad_down) {
                autoMode = AutoMode.JustShoot;
            }

            // select park mode
            if (gamepad1.right_bumper) {
                parkMode = ParkMode.Center;
            } else if (gamepad1.left_bumper) {
                parkMode = ParkMode.Corner;
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
            telemetry.addData("Team", team == Team.RED ? "Red" : "Blue");
            telemetry.addData("Mode", autoMode);
            telemetry.addData("Park", parkMode);
            telemetry.addData("Delay", (double)delayTime/1000);

            if ((gamepad1.left_stick_button && gamepad1.right_stick_button) || isStarted()){
                telemetry.addData("Confirmed!", "");
                if (team == Team.BLUE) {
                    voiFront = voiColorFront;
                    voiBack = voiColorBack;
                } else if (team == Team.RED){
                    voiFront = voiColorBack;
                    voiBack = voiColorFront;
                }
                voiFront.team = driveTrain.team = voiBack.team = team;
                confirmed = true;
                switch (autoMode) {
                    case JustShoot:
                        // Always line up from non beacon corner towards the blue beacons parallel to wall
                        betweenBeacon += 3;
                        bpPower += 0.05;
                        zeroAngle = imu.getAngle();
                        switch (team) {
                            case BLUE:
                                wallAngle = VOIImu.subtractAngles(zeroAngle, 90);
                                shootPower = 0.7;
                                break;
                            case RED:
                                shootPower = 0.7;
                                wallAngle = VOIImu.addAngles(zeroAngle, 180);
                                break;
                        }
                        break;
                    case TwoBall:
                        switch (team) {
                            case RED:
                                wallAngle = imu.getAngle();
                                break;
                            case BLUE:
                                wallAngle = VOIImu.addAngles(wallAngle, 180);
                                break;
                        }
                        break;
                    case ThreeBall:
                        // same for both sides
                        wallAngle = VOIImu.addAngles(imu.getAngle(), 90);
                        break;
                    case VisionBeacon:
                        wallAngle = imu.getAngle();
                        break;
                }
            }
            // Tell color sensor and drive train the team color, which is important for detecting the
            // team color (correctColor) and for driving orientations (powerUp, moveUp, etc)
            voiColorBack.team = voiColorFront.team = driveTrain.team = team;
            intakeTask.setTeam(this.team);
            telemetry.update();

        }
        if (team == BLUE) {
            beaconRotation = 55;
        }
        while (gamepad1.left_stick_button && gamepad1.right_stick_button);
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

    public void pickUpBallAndShoot() {
        intakeTask.visionOn = true;
        driveTrain.moveLeftNInch(1, 24, 5, false, true, true);
        driveTrain.rotateToAngle(VOIImu.addAngles(wallAngle, 90));
        driveTrain.moveRightNInch(1, 12, 3, false, true, false);
        scanWall = true;
        while(opModeIsActive()) {
            // too much to right is too big y, is negative power
            if (cameraNumber == 1) {
                correctPower = -0.03;
            } else {
                correctPower = 0.03;
            }
            if (visMode == PARTICLES) {
                if (strafeAim()) {
                    if (foundTarget())
                        return;
                }
            } else if (visMode == VORTEX) {
                if (rotateAim()) {
                    if (foundTarget())
                        return;
                }
            }
        }
    }

    public Mat processFrame(Mat rgba, Mat gray) {
        if (visMode != prevVisMode) {
            sleep(500);
            prevVisMode = visMode;
            return rgba;
        }
        if (rgba.channels() != 4) {
            return rgba;
        }
        //Callback from OpenCV leads here
        //Do image processing for individual frames here
        //Convert image to HSV format
        if (explore) {
            mRgbaMat = rgba;
            //Define two color ranges to match as red because the hue for red crosses over 180 to 0
            if (visMode == PARTICLES) {
                rgba = findCircles(rgba, gray);
            } else {
                rgba = findVortex(rgba, gray);
            }
            //System.out.println("X: " + centX + " Y: " + centY);
            //System.out.println("Vortex bottom: " + vortexBottom);
            Drawing.drawRectangle(rgba, new Point(1, 1), new Point(IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1), new ColorRGBA(255, 255, 255));
        }
        return rgba;
    }

    public void initVision() {
        mHsvMat = new Mat();
        red1 = new Mat();
        red2 = new Mat();
        colorMask = new Mat();
        blurred = new Mat();
        convexHull = new MatOfInt();
        temp2fMat = new MatOfPoint2f();
        polyApprox = new MatOfPoint2f();
        convexityDefects = new MatOfInt4();
        initialContourList = new ArrayList<>();
        potentialContours = new ArrayList<>();
        passedFirstCheck = new ArrayList<>();
        resultContours = new ArrayList<>();
        grayContours = new ArrayList<>();
        grayMask = new Mat();
        lowValMask = new Mat();
        lowSatMask = new Mat();

    }

    public boolean rotateAim() {
        double target = 0;
        if (visMode == VORTEX) {
            target = VORTEX_TARGET;
        } else if (visMode == PARTICLES) {
            target = PARTICLE_TARGET;
        }
        if (centY != -1) {
            if (pauseTimer.time() < 500) {
                driveTrain.stopAll();
                sleep(100);
                pauseTimer.reset();
            }
        }
        if (centY == -1) {
            ballDetected = false;
            if (team == BLUE) {
                driveTrain.startRotation(0.05);
            } else if (team == RED) {
                driveTrain.startRotation(-0.05);
            }
        } else if (centY < target - ACCEPTABLE_ERROR) {
            ballDetected = false;
            driveTrain.startRotation(correctPower);
        } else if (centY > target + ACCEPTABLE_ERROR) {
            ballDetected = false;
            driveTrain.startRotation(-correctPower);
        } else {
            driveTrain.stopAll();
            return true;
        }
        return false;
    }

    public boolean strafeAim() {
        double target = 0;
        if (visMode == VORTEX) {
            target = VORTEX_TARGET;
        } else if (visMode == PARTICLES) {
            target = PARTICLE_TARGET;
        }
        if (centY != -1) {
            if (pauseTimer.time() < 500) {
                driveTrain.stopAll();
                sleep(100);
                pauseTimer.reset();
            }
        }
        driveTrain.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (centY == -1) {
            ballDetected = false;
            if (team == RED) {
                driveTrain.strafeLeft(0.8);
            } else if (team == BLUE) {
                driveTrain.strafeRight(0.8);
            }
        } else if (centY < target - ACCEPTABLE_ERROR) {
            ballDetected = false;
            driveTrain.strafeRight(0.6);
        } else if (centY > target + ACCEPTABLE_ERROR) {
            ballDetected = false;
            driveTrain.strafeLeft(0.6);
        } else {
            driveTrain.stopAll();
            driveTrain.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }
        return false;
    }

    public boolean foundTarget() {
        if (!ballDetected) {
            driveTrain.stopAll();
        }
        if (visMode == PARTICLES) {
            sleep(250);
            if (Math.abs(centY - PARTICLE_TARGET) < ACCEPTABLE_ERROR) {
                driveForward();
            }
        } else if (visMode == VORTEX) {
            if (distanceAim2()) {
                flywheelTask.setPhoneDown();
                shoot();
                driveTrain.moveBackwardNInch(0.5, shootDistance, 5, false, true, false);
                return true;
            }
        }
        return false;
    }

    public boolean distanceAim() {
        System.out.println("DISTANCE AIM");
        System.out.println("Initial Position");
        printPosition();
        if (centX == -1) {
            driveTrain.stopAll();
            return false;
        }
        double dist = distance (robotX, vortexX, robotY, vortexY);
        double distToDrive = dist - shootDistance;
        if ( distToDrive > 0) {
            double initialAngle = driveTrain.getAngle();
            driveTrain.moveBackwardNInch(0.15, distToDrive, 5, false, true, false);
            robotX -= distToDrive * Math.cos(initialAngle*Math.PI/180);
            robotY -= distToDrive * Math.sin(initialAngle*Math.PI/180);
            validateCoordinates();
        } else if (dist < 0){
            double initialAngle = driveTrain.getAngle();
            driveTrain.moveForwardNInch(0.15, Math.abs(distToDrive), 5, false, true, false);
            robotX -= distToDrive * Math.cos(initialAngle*Math.PI/180);
            robotY -= distToDrive * Math.sin(initialAngle*Math.PI/180);
            validateCoordinates();
        }
        System.out.println("Final Position: ");
        printPosition();
        driveTrain.stopAll();
        while (!rotateAim() && opModeIsActive());
        printPosition();
        return true;

    }

    public boolean distanceAim2() {
        while (opModeIsActive()) {
            if (gameTimer.time() > 25000) {
                driveTrain.stopAll();
                shoot();
                return true;
            }
            if (vortexBottom == -1) {
                driveTrain.stopAll();
                return false;
            } else if (vortexBottom > TARGET_VORTEX_HEIGHT + 50) {
                driveTrain.powerAllMotors(0.08);
            } else if (centX < TARGET_VORTEX_HEIGHT - 50) {
                driveTrain.powerAllMotors(-0.08);
            } else {
                driveTrain.stopAll();
                while (!rotateAim() && opModeIsActive());
                return true;
            }
        }
        return false;
    }

    double distance(double x1, double x2, double y1, double y2) {
        double x = x1 - x2;
        double y = y1 - y2;
        return Math.sqrt(x*x + y*y);
    }

    public void driveForward() {
        while (opModeIsActive() && gameTimer.time() < 10000);
        ballDetected = true;
        double driveAngle = driveTrain.getAngle();
        // because the angle we get is clockwise, we need to make it negative to get the correct sine
        double angle = -VOIImu.subtractAngles(driveAngle, zeroAngle);
        if (centX > IMAGE_WIDTH * 0.7) {
            // if ball is close enough
            flywheelTask.setFlywheelPow(shootPower);
            int startTicks = driveTrain.backRight.getCurrentPosition();
            intakeTask.setPower(1);
            driveTrain.powerAllMotors(0.3);
            rejectTimer.reset();
            explore = false;
            changeVisMode(VORTEX);
            timer.reset();

            while (opModeIsActive() && rejectTimer.time() < 3000) {
                // 3000 ms timeout
                if (intakeTask.correctColor()) {
                    // when ball is detected
                    driveTrain.stopAll();
                    sleep(200);
                    // average before and after angles in case robot gets off path (not sure if it works)
                    /*
                    double driveAngle2 = -driveTrain.getAngle();
                    double finalAngle = VOIImu.subtractAngles(driveAngle2, zeroAngle);
                    double diff = VOIImu.subtractAngles(finalAngle, angle);
                    angle = VOIImu.addAngles(angle, diff);*/
                    double totalInches = (driveTrain.backRight.getCurrentPosition() - startTicks)/MecanumDriveTrain.TICKS_PER_INCH_FORWARD;

                    updatePosition(totalInches, angle);
                    validateCoordinates();
                    double vortexAngle = VOIImu.subtractAngles(Math.atan2(vortexY - robotY, vortexX - robotX) * 180/Math.PI, zeroAngle);
                    printPosition();
                    System.out.println("Vortex angle: " + vortexAngle);
                    if (!scanWall) {
                        driveTrain.rotateToAngle(VOIImu.subtractAngles(180, vortexAngle));
                        while (distance(robotX, vortexX, robotY, vortexY) < 50) {
                            double tempAngle = imu.getAngle();
                            driveTrain.moveForwardNInch(0.3, 5, 5, false, true, false);
                            updatePosition(20, tempAngle);
                        }
                    } else {
                        driveTrain.moveBackwardNInch(0.5, 5, 3, false, true, false);
                    }
                    explore = true;
                    if (!scanWall) {
                        while (timer.time() < 3500 && opModeIsActive()) ;
                    } else {
                        while (timer.time() < 1500 && opModeIsActive()) ;
                    }
                    centY = centX = -1;
                    return;
                }
            }
            explore = true;
        } else {
            driveTrain.moveForwardNInch(0.3, 8, 5, false, true, false);
            double driveAngle3 = -driveTrain.getAngle();
            double finalAngle = VOIImu.subtractAngles(driveAngle3, zeroAngle);
            double diff = VOIImu.subtractAngles(finalAngle, angle);
            angle = VOIImu.addAngles(angle, diff);
            robotX += 10*VOIImu.cosine(angle);
            robotY += 10*VOIImu.sine(angle);
            validateCoordinates();
        }
        printPosition();
        checkViolatingBeacon();
    }

    public void changeCamera(int a) {
        if (cameraNumber != a) {
            cameraNumber = a;
            stopCamera();
            initCamera(a);
            initVision();
        }
    }

    public void tileFilter(Mat rgba) {
        Mat lowMask = new Mat();
        Mat highMask = new Mat();
        Core.inRange(mHsvMat, new Scalar(0, 50, 50), new Scalar (50, 255, 255), lowMask);
        Core.inRange(mHsvMat, new Scalar (140, 50, 50), new Scalar(180, 255, 255), highMask);
        Core.inRange(mHsvMat, new Scalar(0, 175, 0), new Scalar(255, 255, 255), lowSatMask);
        Core.inRange(mHsvMat, new Scalar(0, 0, 175), new Scalar(255, 255, 255), lowValMask);
        Core.bitwise_or(lowSatMask, lowValMask, grayMask);
        Core.bitwise_or(lowMask, grayMask, grayMask);
        Core.bitwise_or(highMask, grayMask, grayMask);
        Imgproc.findContours(grayMask, grayContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < grayContours.size(); i ++) {
            MatOfPoint p = grayContours.get(i);
            Rect rect = Imgproc.boundingRect(p);
            int n = rect.x + rect.width/2;
            if (Imgproc.contourArea(p) > PARTICLE_MAX_THRESHOLD) {
                Imgproc.drawContours(rgba, grayContours, i, blackScalar, -1);
            }
        }
        Drawing.drawContours(rgba, resultContours, new ColorRGBA(0, 0, 255));

        resultContours.clear();
        grayContours.clear();
    }

    void optionsVision() {
        boolean confirmed = false;
        boolean upPressed = false;
        boolean downPressed = false;

        int modify = 0;
        int diff = 0;
        while (!confirmed && !isStopRequested()) {

            if (gamepad1.dpad_up && !upPressed) {
                upPressed = true;
                diff = 2;
            }
            if (gamepad1.dpad_down && !downPressed) {
                downPressed = true;
                diff = -2;
            }
            if (!gamepad1.dpad_up) {
                upPressed = false;
            }
            if (!gamepad1.dpad_down) {
                downPressed = false;
            }
            if (gamepad1.y) {
                modify = 0;
            }
            if (gamepad1.b) {
                modify = 1;
            }
            if (gamepad1.a) {
                modify = 2;
            }
            if (gamepad1.x) {
                modify = 3;
            }
            if (gamepad1.right_bumper) {
                modify = 4;
            }
            if (gamepad1.left_bumper) {
                modify = 5;
            }
            if (gamepad1.right_trigger > 0.2) {
                modify = 6;
            }
            if (gamepad1.left_trigger > 0.2) {
                modify = 7;
            }
            if (gamepad1.right_stick_x < -0.5) {
                changeVisMode(VORTEX);
            }
            if (gamepad1.right_stick_x > 0.5) {
                changeVisMode(PARTICLES);
            }
            if (gamepad1.left_stick_x < -0.5) {
                team = RED;
            }
            if (gamepad1.left_stick_x > 0.5) {
                team = BLUE;
            }
            switch (modify) {
                case 0:
                    telemetry.addData("Modifying", "Min Blue");
                    minBlueH += diff;
                    diff = 0;
                    break;
                case 1:
                    telemetry.addData("Modifying", "Max Blue");
                    maxBlueH += diff;
                    diff = 0;
                    break;
                case 2:
                    telemetry.addData("Modifying", "Min Red Hue");
                    minRedH += diff;
                    diff = 0;
                    break;
                case 3:
                    telemetry.addData("Modifying", "Max Red Hue");
                    maxRedH += diff;
                    diff = 0;
                    break;
                case 4:
                    telemetry.addData("Modifying", "Min Blue Sat");
                    minBlueSat += diff;
                    diff = 0;
                    break;
                case 5:
                    telemetry.addData("Modifying", "Min Red Sat");
                    minRedSat += diff;
                    diff = 0;
                    break;
                case 6:
                    telemetry.addData("Modifying", "Min Blue Val");
                    minBlueVal += diff;
                    diff = 0;
                    break;
                case 7:
                    telemetry.addData("Modifying", "Min Red Val");
                    minRedVal += diff;
                    diff = 0;
                    break;

            }

            telemetry.addData("Min Blue", minBlueH);
            telemetry.addData("Max Blue", maxBlueH);
            telemetry.addData("Min Red H", minRedH);
            telemetry.addData("Max Red H", maxRedH);
            telemetry.addData("Min Blue Sat", minBlueSat);
            telemetry.addData("Min Red Sat", minRedSat);
            telemetry.addData("Min Blue Val", minBlueVal);
            telemetry.addData("Min Red Val", minRedVal);
            telemetry.addData("VisMode", visMode);
            telemetry.addData("Team", team);

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                confirmed = true;
                telemetry.addData("Confirmed!", "");
            }
            telemetry.update();

        }
        while (gamepad1.left_stick_button && gamepad1.right_stick_button);
        if (team == BLUE)
            zeroAngle = VOIImu.addAngles(wallAngle, 90);
        else if (team == RED) {
            zeroAngle = VOIImu.addAngles(wallAngle, 180);
        }
    }

    boolean possibleBall(double area, int x) {
        if (cameraNumber == 1) {
            x = IMAGE_WIDTH - x;
        }
        return area > PARTICLE_MIN_THRESHOLD && area < Math.max(PARTICLE_MAX_THRESHOLD * x/IMAGE_WIDTH, 2000);
    }

    void changeVisMode (VisionShootingTest.VisionMode visMode) {
        if (visMode != this.visMode) {
            centX = centY = -1;
            this.visMode = visMode;
            if (visMode == PARTICLES) {
                changeCamera(0);
            } else {
                changeCamera(1);
            }
        }
    }

    void startRobot() {
        intakeTask.start();
        flywheelTask.start();
        flywheelTask.setPhoneVision();
    }

    Mat findCircles(Mat rgba, Mat gray) {
        if (rgba == null) {
            return rgba;
        }

        //Imgproc.GaussianBlur(rgba, rgba, new Size(11, 11), 5, 5);
        Imgproc.cvtColor(rgba, mHsvMat, Imgproc.COLOR_RGB2HSV);
        Mat filtered = new Mat();
        if (team == BLUE) {
            Core.inRange(mHsvMat, new Scalar(minBlueH, minBlueSat, minBlueVal), new Scalar(maxBlueH, 255, 255), filtered);
        } else if (team == RED) {
            Core.inRange(mHsvMat, new Scalar(minRedH, minRedSat, minRedVal), new Scalar(maxRedH, 255, 255), filtered);
            Core.inRange(mHsvMat, new Scalar(0, minRedSat, minRedVal), new Scalar(15, 255, 255), red1);
            Core.bitwise_or(filtered, red1, filtered);
        }
        Imgproc.findContours(filtered, initialContourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double minCircleArea = PARTICLE_MIN_THRESHOLD;
        for (MatOfPoint p : initialContourList) {
            double contourArea = Imgproc.contourArea(p);
            Rect rect = Imgproc.boundingRect(p);
            double radius = (rect.width + rect.height)/4;
            double circleArea = Math.PI * radius * radius;
            int centerX = rect.x + rect.width/2;
            int centerY = rect.y + rect.height/2;
            if (centerX < IMAGE_WIDTH/2) {
                continue;
            }
            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 0, 255));
            if (centX != -1) {
                if (Math.abs(centerX - centX) > 100 || Math.abs(centerY - centY) > 100) {
                    continue;
                }
            } else {
                if ((double)rect.width/rect.height > 1.3 || (double)rect.height/rect.width > 1.3) {
                    continue;
                }
            }

            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 0, 0));

            if (!possibleBall(contourArea, centerX)) {
                continue;
            }
            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 255, 255));

            if (contourArea/circleArea > 0.7) {
                Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 0));
                Drawing.drawCircle(rgba, new Point(rect.x + rect.width/2, rect.y + rect.height/2), (int)(radius),new ColorRGBA(255, 0, 0) );
                if (contourArea > minCircleArea && possibleBall(contourArea, centerX)) {
                    minCircleArea = contourArea;
                    centX = centerX;
                    centY = centerY;
                    passedFirstCheck.clear();
                    passedFirstCheck.add(p);
                }
            }
        }
        if (passedFirstCheck.isEmpty()) {
            centX = centY = -1;
        } else {
            Drawing.drawCircle(rgba,new Point(centX, centY), 10, new ColorRGBA(0, 255, 0));
        }
        initialContourList.clear();
        passedFirstCheck.clear();
        return rgba;
    }

    Mat findVortex(Mat rgba, Mat gray) {
        if (rgba == null) {
            return rgba;
        }
        //Imgproc.GaussianBlur(rgba, rgba, new Size(5, 5), 2, 2);
        Imgproc.cvtColor(rgba, mHsvMat, Imgproc.COLOR_RGB2HSV);
        Mat filtered = new Mat();
        Mat filtered2 = new Mat();
        if (team == BLUE) {
            Core.inRange(mHsvMat, new Scalar(minVortexBlueH, minVortexBlueSat, 15), new Scalar(maxVortexBlueH, 255, maxVortexBlueVal), filtered);
        } else if (team == RED) {
            Core.inRange(mHsvMat, new Scalar(minRedH, minRedSat, minRedVal), new Scalar(255, 255, 255), filtered);
            Core.inRange(mHsvMat, new Scalar(0, minRedSat, minRedVal), new Scalar(15, 255, 255), filtered2);
            Core.bitwise_or(filtered, filtered2, filtered);
        }
        //Imgproc.GaussianBlur(filtered, filtered, new Size(5, 5), 10);
        initialContourList.clear();
        potentialContours.clear();
        passedFirstCheck.clear();
        resultContours.clear();
        //Find the external contours for the red mask using the fast simple approximation
        Imgproc.findContours(filtered, initialContourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double minArea = VORTEX_THRESHOLD;
        for(MatOfPoint p: initialContourList) {        //Go through preliminary list of contours
            p.convertTo(temp2fMat, CvType.CV_32F);      //Convert MatOfPoint to MatofPoint2f to run approxPolyDP
            double perimeter = Imgproc.arcLength(temp2fMat, true);
            //Approximate each contour using a polygon
            double contourArea = Imgproc.contourArea(p);
            Imgproc.approxPolyDP(temp2fMat, polyApprox, 0.02*perimeter, true);
            MatOfPoint polyApproxFloat = new MatOfPoint(polyApprox.toArray());
            Rect rect = Imgproc.boundingRect(polyApproxFloat);
            int y = rect.y + rect.height/2;
            int x = rect.x + rect.width/2;

            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 0, 255));
            if (centX != -1) {
                if (Math.abs(x-centX) > 100 || Math.abs(y-centY) > 100) {
                    continue;
                }
            }
            if (x < IMAGE_WIDTH/2) {
                continue;
            }

            if(polyApproxFloat.toArray().length > 4) {
                // Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(0, 255, 0));
                if(contourArea > minArea) {
                    // Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 0, 0));

                    Imgproc.convexHull(polyApproxFloat, convexHull);
                    if(convexHull.rows() > 2) {
                        // Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 0));
                        Imgproc.convexityDefects(polyApproxFloat, convexHull, convexityDefects);
                        List<Integer> cdlist = convexityDefects.toList();
                        int count = 0;
                        for(int i = 0; i < cdlist.size(); i+=4) {
                            double depth = cdlist.get(i+3)/256.0;
                            if (depth > (rect.height)/2) {
                                count++;
                            }
                        }
                        if (count > 0) {
                            minArea = contourArea;
                            passedFirstCheck.clear();
                            passedFirstCheck.add(p);
                            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 255));
                        }
                    }
                } else {
                    potentialContours.add(p);
                }
            }
        }
        // take only the largest contour
        while (passedFirstCheck.size() > 1) {
            double size0 = Imgproc.contourArea(passedFirstCheck.get(0));
            double size1 = Imgproc.contourArea(passedFirstCheck.get(1));
            if (size0 > size1) {
                passedFirstCheck.remove(1);
            } else {
                passedFirstCheck.remove(0);
            }
        }


        if (passedFirstCheck.isEmpty()) {
            vortexBottom = centY = centX =-1;
        } else {
            MatOfPoint p = passedFirstCheck.get(0);
            Rect rect = Imgproc.boundingRect(p);
            resultContours.add(new Contour(p));
            centX = rect.x + rect.width/2;
            centY = rect.y + rect.height/2;
            vortexBottom = rect.x;
        }
        Drawing.drawContours(rgba, resultContours, new ColorRGBA(255, 0, 0), 2);
        Drawing.drawCircle(rgba, new Point(centX, centY), 10, new ColorRGBA(255, 255, 255));
        return rgba;
    }

    void printPosition() {
        System.out.println("Position: X: " + robotX + " Y: " + robotY);
    }

    public void setInitialPosition() {
        int changeValue = 1;
        boolean confirmed = false;
        boolean upPressed = false;
        boolean downPressed = false;
        boolean leftPressed = false;
        boolean rightPressed = false;
        boolean rBumper = false;
        boolean lBumper = false;
        zeroAngle = driveTrain.getAngle();
        while (!confirmed) {
            if (gamepad1.dpad_left && !leftPressed) {
                robotX -= changeValue;
                leftPressed = true;
            }
            if (gamepad1.dpad_right && !rightPressed) {
                robotX += changeValue;
                rightPressed = true;
            }
            if (gamepad1.dpad_up && !upPressed) {
                robotY += changeValue;
                upPressed = true;
            }
            if (gamepad1.dpad_down && !downPressed) {
                robotY -= changeValue;
                downPressed = true;
            }
            if (gamepad1.right_bumper && !rBumper) {
                rBumper = true;
                changeValue ++;
            }
            if (gamepad1.left_bumper && !lBumper) {
                lBumper = true;
                changeValue --;
            }
            if (!gamepad1.dpad_left) {
                leftPressed = false;
            }
            if (!gamepad1.dpad_right) {
                rightPressed = false;
            }
            if (!gamepad1.dpad_up) {
                upPressed = false;
            }
            if (!gamepad1.dpad_down) {
                downPressed = false;
            }
            if (!gamepad1.right_bumper) {
                rBumper = false;
            }
            if (!gamepad1.left_bumper) {
                lBumper = false;
            }
            telemetry.addData("Change Value", changeValue);
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                confirmed = true;
                telemetry.addData("Confirmed!","");
            }
            telemetry.update();

        }
        while (gamepad1.left_stick_button && gamepad1.right_stick_button);
    }

    public void coolDown() {
        intakeTask.setPower(0);
        flywheelTask.setFlywheelPow(0);
    }

    void validateCoordinates() {
        robotX = Math.max(0, robotX);
        robotY = Math.max(0, robotY);
        robotX = Math.min(144, robotX);
        robotY = Math.min(144, robotY);
    }

    void shootFromCorner(double inches) {
        driveTrain.moveBackwardNInch(0.2, inches, 10, false, true, false);
        flywheelTask.setFlywheelPow(shootPower);
        shoot();
    }

    void runVisionShoot() {
        shootFromCorner(26);
        flywheelTask.setPhoneVision();
        driveTrain.moveForwardNInch(0.7, 20, 10, false, true, false);
        if (team == BLUE)
            driveTrain.rotateToAngle(wallAngle);
        else if (team == RED) {
            //driveTrain.rotateToAngle(VOIImu.addAngles(wallAngle, 180));
            driveTrain.rotateToAngle(zeroAngle);
        }

        intakeTask.visionOn = true;
        while (opModeIsActive()) {
            if (cameraNumber == 1) {
                correctPower = -0.03;
            } else {
                correctPower = 0.03;
            }
            if (rotateAim()) {
                visionActive = true;
                if (foundTarget())
                    return;
            }
            if (gameTimer.time() > 23000 && !visionActive) {
                driveTrain.rotateToAngle(aimVortexAngle);
                driveTrain.moveBackwardNInch(0.3, 55, 7, false, true, false);
                break;
            }

        }

    }

    void updatePosition(double inches, double angle) {
        System.out.println("Inches: " + inches + " Angle: " + angle);
        double xComponent = inches * VOIImu.cosine(angle);
        double yComponent = inches * VOIImu.sine(angle);
        System.out.println("Delta X: " + xComponent + " Delta Y: " + yComponent);
        robotX += xComponent;
        robotY += yComponent;
    }

    void checkViolatingBeacon() {
       if (robotX > 120 || robotY > 120) {
            double driveAngle = VOIImu.subtractAngles(imu.getAngle(), zeroAngle);
            driveTrain.moveBackwardNInch(0.5, 30, 5, false, true, false);
            updatePosition(-30, driveAngle);
        }
    }

}