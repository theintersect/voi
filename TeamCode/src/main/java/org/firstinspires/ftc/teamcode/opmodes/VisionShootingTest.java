package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotutil.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.robotutil.Team;
import org.firstinspires.ftc.teamcode.robotutil.VOIImu;
import org.firstinspires.ftc.teamcode.tasks.ButtonPusherTask;
import org.firstinspires.ftc.teamcode.tasks.CapBallTask;
import org.firstinspires.ftc.teamcode.tasks.FlywheelTask;
import org.firstinspires.ftc.teamcode.tasks.IntakeTask;
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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.opmodes.VisionShootingTest.VisionMode.PARTICLES;
import static org.firstinspires.ftc.teamcode.opmodes.VisionShootingTest.VisionMode.VORTEX;
import static org.firstinspires.ftc.teamcode.robotutil.Team.BLUE;
import static org.firstinspires.ftc.teamcode.robotutil.Team.RED;


/**
 * Created by Stephen on 12/23/2016.
 * Vision Shooting Test
 */

/*
    OpenCV Testing: Uses the LinearOpModeVision class that is supposed to gather image data, supposed to display the images from OpenCV to a camera view on the screen. Untested!!
 */

@SuppressWarnings("ConstantConditions")
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "VisionShootingTest", group = "Tests")

public class VisionShootingTest extends LinearOpModeVision {

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

    FlywheelTask flywheelTask;
    IntakeTask intakeTask;
    Servo phoneServo;

    boolean detectedVortex = false;

    int centX = -1;
    int centY = -1;
    double zeroAngle = 0;

    static final int TARGET_VORTEX_HEIGHT = 360;
    static final int VORTEX_THRESHOLD = 8000;
    static final int IMAGE_HEIGHT = 600;       //ZTE Camera picture size
    static final int IMAGE_WIDTH = 800;
    static final double MAX_VORTEX_AREA_RATIO = 0.6;

    static final double visionPosition = 0.2, downPostion = 0, restPosition = 0.7;

    static final int ACCEPTABLE_ERROR = 40;
    static final int PARTICLE_TARGET = 300;
    static final int VORTEX_TARGET = 350;
    static final double MIN_PARTICLE_AREA_RATIO = 0.6;
    static final double PARTICLE_MIN_THRESHOLD = 500;
    static final double PARTICLE_MAX_THRESHOLD = 20000;
    double correctPower = 0.03;

    double[] blackArray = new double[] {0, 0, 0, 0};
    Scalar blackScalar = new Scalar(0, 0, 0);
    double vortexBottom = -1;

// lower value , more to the right for balls
    // this is for vision
    static int minVortexBlueH = 96;
    static int maxVortexBlueH = 120;
    static int minVortexBlueSat = 50;
    static int maxVortexBlueVal = 200;

    static int minBlueH = 80;
    static int maxBlueH = 110;
    static int minRedH = 160;
    static int maxRedH = 240;
    static int minBlueSat = 125;
    static int minRedSat = 50;
    static int minBlueVal = 75;
    static int minRedVal = 50;

    int cameraNumber = 0;
    MecanumDriveTrain driveTrain;
    boolean detectedTarget = false;
    ElapsedTime rejectTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int rejectTime = 500;
    ElapsedTime pauseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    Team team = BLUE;
    VisionMode visMode = PARTICLES;
    VisionMode prevVisMode = PARTICLES;

    boolean ballDetected = false;
    boolean explore = true;

    // location stuff
    double robotX = 36;
    double robotY = 24;
    double robotAngle = 0;
    double vortexX = 80;
    double vortexY = 64;

    // general autonomous stuff
    private int shootTime = 2500;

    final double shootDistance = 36;

    public enum VisionMode {
        VORTEX, PARTICLES
    }

    @Override
    public void runOpMode() {
        initCamera(cameraNumber);   //Start OpenCV
        initVision();   //Do a bunch of initialization for vision code
        //initRobot();
        options();
        setInitialPosition();
        waitForStart();
        changeVisMode(PARTICLES);
        startRobot();

        ElapsedTime t = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
        stopCamera();   //Tear down the camera instance
        System.out.println("Camera Stopped");
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        if (visMode != prevVisMode) {
            sleep(500);
            prevVisMode = visMode;
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
            System.out.println("X: " + centX + " Y: " + centY);
            //System.out.println("Vortex bottom: " + vortexBottom);
            Drawing.drawRectangle(rgba, new Point(1, 1), new Point(IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1), new ColorRGBA(255, 255, 255));
        }
        return rgba;
    }

    public void initRobot() {
        //Initialize robot hardware and stuff here
        driveTrain = new MecanumDriveTrain(this);
        intakeTask = new IntakeTask(this);
        new CapBallTask(this);
        new ButtonPusherTask(this);
        flywheelTask = new FlywheelTask(this);
        flywheelTask.setPhoneVision();
        zeroAngle = driveTrain.getAngle();
        intakeTask.visionOn = true;
        if (team == RED) {
            double temp = vortexX;
            //noinspection SuspiciousNameCombination
            vortexX = vortexY;
            vortexY = temp;
        }

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
            driveTrain.startRotation(0.04);
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
        ballDetected = true;
        double angle = -VOIImu.subtractAngles(driveTrain.getAngle(), zeroAngle);
        if (centX > IMAGE_WIDTH * 0.7) {
            flywheelTask.setFlywheelPow(0.8);
            int startTicks = driveTrain.backRight.getCurrentPosition();
            intakeTask.setPower(1);
            driveTrain.powerAllMotors(0.3);
            rejectTimer.reset();
            explore = false;
            changeVisMode(VORTEX);
            timer.reset();

            while (opModeIsActive() && rejectTimer.time() < 3000) {
                if (intakeTask.correctColor()) {
                    driveTrain.stopAll();
                    sleep(200);
                    double finalAngle = -VOIImu.subtractAngles(driveTrain.getAngle(), zeroAngle);
                    double diff = VOIImu.subtractAngles(finalAngle, angle);
                    angle = VOIImu.addAngles(angle, diff);
                    double totalInches = (driveTrain.backRight.getCurrentPosition() - startTicks)/MecanumDriveTrain.TICKS_PER_INCH_FORWARD;
                    robotX += totalInches * VOIImu.cosine(angle);
                    robotY += totalInches * VOIImu.sine(angle);
                    validateCoordinates();
                    printPosition();
                    double vortexAngle = VOIImu.subtractAngles(Math.atan2(vortexY - robotY, vortexX - robotX) * 180/Math.PI, zeroAngle);
                    printPosition();
                    System.out.println("Vortex angle: " + vortexAngle);
                    driveTrain.rotateToAngle(VOIImu.subtractAngles(180,vortexAngle));
                    explore = true;
                    while (timer.time() < 3500 && opModeIsActive());
                    return;
                }
            }
            explore = true;
        } else {
            driveTrain.moveForwardNInch(0.3, 8, 5, false, true, false);
            double finalAngle = -VOIImu.subtractAngles(driveTrain.getAngle(), zeroAngle);
            double diff = VOIImu.subtractAngles(finalAngle, angle);
            angle = VOIImu.addAngles(angle, diff);
            robotX += 10*VOIImu.cosine(angle);
            robotY += 10*VOIImu.sine(angle);
            validateCoordinates();
        }
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

    void options() {
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
        zeroAngle = driveTrain.getAngle();
    }

    boolean possibleBall(double area, int x) {
        if (cameraNumber == 1) {
            x = IMAGE_WIDTH - x;
        }
        return area > PARTICLE_MIN_THRESHOLD && area < Math.max(PARTICLE_MAX_THRESHOLD * x/IMAGE_WIDTH, 2000);
    }

    void changeVisMode (VisionMode visMode) {
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
        /*List<Mat> channels = new ArrayList<>();
        Core.split(mHsvMat, channels);
        Mat hue = channels.get(0);
        Mat sat = channels.get(1);
        Mat val = channels.get(2);
        if (team == BLUE) {
            Core.inRange(hue, new Scalar(minBlueH), new Scalar(maxBlueH), hue);
            Core.inRange(sat, new Scalar(minBlueSat), new Scalar(255), sat);
            Core.inRange(val, new Scalar(minBlueVal), new Scalar(255), val);
        } else if (team == RED) {
            Core.inRange(hue, new Scalar(minRedH), new Scalar(maxRedH), hue);
            Core.inRange(sat, new Scalar(minRedSat), new Scalar(255), sat);
            Core.inRange(val, new Scalar(minRedVal), new Scalar(255), val);
        }*/
                Mat filtered = new Mat();
        Mat filtered2 = new Mat();
        if (team == BLUE) {
            Core.inRange(mHsvMat, new Scalar(minVortexBlueH, minVortexBlueSat, 15), new Scalar(maxVortexBlueH, 255, maxVortexBlueVal), filtered);
        } else if (team == RED) {
            Core.inRange(mHsvMat, new Scalar(minRedH, minRedSat, minRedVal), new Scalar(maxRedH, 255, 255), filtered);
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

            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 255, 255));
            if (centX != -1) {
                if (Math.abs(x-centX) > 100 || Math.abs(y-centY) > 100) {
                    continue;
                }
            }
            if (x < IMAGE_WIDTH/3) {
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
                            Drawing.drawRectangle(rgba, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new ColorRGBA(255, 0, 0));
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
            //noinspection SuspiciousNameCombination
            vortexBottom = rect.x;
        }
        Drawing.drawContours(rgba, resultContours, new ColorRGBA(0, 0, 255), 2);
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
        while (!confirmed && !isStopRequested()) {
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

    public void shoot() {
        flywheelTask.setPhoneDown();
        System.out.println("Shoot");
        timer.reset();
        /*intakeTask.oscillate = true;
        int count = -100;
        while (opModeIsActive()) {
            if (flywheelTask.getFlywheelState() == FlywheelTask.FlywheelState.STATE_RUNNING_NEAR_TARGET) {
                if (flywheelTask.count == count + 2) {
                    System.out.println(flywheelTask.count + " Good");
                    break;
                } else {
                    if (count == -100) {
                        count = flywheelTask.count;
                        System.out.println(count + " Good");
                    }
                }
            } else {
                count = -100;
            }
            if (timer.time() > 5000) {
                System.out.println("Flywheel time out");
                break;
            }
        }
        intakeTask.oscillate = false;
        intakeTask.power = 0;*/
        sleep(500);
        System.out.println("Start shooting");
        intakeTask.setPower(1);
        timer.reset();
        while (timer.time() < shootTime && opModeIsActive());
        coolDown();
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
}

