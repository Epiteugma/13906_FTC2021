package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.visionv1.*;

import org.firstinspires.ftc.teamcode.events.EventDispatcher;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.List;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class Robot {
    TseDetector detector;
    Thread armThread;
    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    Motor frontRight;
    Motor frontLeft;
    Motor backRight;
    Motor backLeft;
    Motor collector;
    Motor arm;
    Motor duckSpinner;
    RevIMU imu;
    SensorColor cargoDetector;
    SensorRevTOFDistance frontDistance;
    double armPower = 0;
    boolean armHoldPosition = false;
    
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    // Constants and ratios
    public static double driveGearRatio = 20.0/10.0;
    public static double driveTicksPerRev = 1120.0 * driveGearRatio;
    public static double armTickPerRev = 1120.0;
    // TODO: adjust gain (almost done)
    public static double driveGain = 0.05;
    public static double strafeGain = 0.125;
    public static double turnGain = 0.1;
    public int duckSpinnersStartPos = 0;
    // Old Wheels
    // public static double wheelRadius = 7/2;
    // New wheels
    public static double wheelRadius = 7.5;
    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
    public static double centerToWheel = 21;
    public static double turnCircumference = 2 * Math.PI * centerToWheel;
    public static double normalFullPowerVelocity = 2700;

    // Ticks Angles and positions
    public int lastClawPosition = 0;
    public double targetRotations = 0;
    public double targetTicks = 0;
    public double ticksToTurn = 0;
    public double correction = 0;
    public double targetAngle = 0;
    public double currentAngle = 0;
    public int currentTicks = 0;

    // Powers and error tolerance
    public double driveErrorTolerance = 5; // in ticks
    public double correctedCappedPower = 0;
    public double cappedPower = 0;
    public double frPower;
    public double flPower;
    public double brPower;
    public double blPower;

    // global ticks
    public int frCurrentTicks;
    public int flCurrentTicks;
    public int brCurrentTicks;
    public int blCurrentTicks;

    // Barriers distance to wall
    public static double afterBarriers = 55;
    public static double beforeBarriers = 819;


    // cargoDetector
    public String detectedCargo = "None";
    public double cubeHeight= 5.08;
    public double ballHeight = 6.99;
    public double duckHeight = 5.4;
    public double collectorBoxHeight = 15;

    public double intakeLowSpeed = 0.5;
    public double intakeMidSpeed = 0.65;
    public double intakeHighSpeed = 0.65;



    private Telemetry telemetry;

    public enum Axis {
        X,
        Y,
        Z
    }

    public enum Direction {
        RIGHT,
        LEFT,
        FORWARDS,
        BACKWARDS,
        IN,
        OUT
    }

    public enum Position {
        LOW(-470),
        MID(-1100),
        HIGH(-1700),
        DOWN(0);

        public final int label;
        Position(int label) {
            this.label = label;
        }
    }
    
    public double getIMUAngle(Axis axis) {
        double[] angles = imu.getAngles();
        switch(axis) {
            case X:
                return angles[0];
            case Y:
                return angles[1];
            case Z:
                return angles[2];
        }
        return 0;
    }

//    public double getIMUAccel(Axis axis) {
//        doubl angles = imu.getHeading();
//        switch(axis) {
//            case X:
//                return angles[0];
//            case Y:
//                return angles[1];
//            case Z:
//                return angles[2];
//        }
//        return 0;
//    }

    // OLD!!!
//    public DuckDetector.Location getDuckPos() {
//        DuckDetector detector;
//        detector = new DuckDetector();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(detector);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//        DuckDetector.Location location = detector.getLocation();
//        return location;
//    }

    public TseDetector.Location getTsePos() {

        return detector.getLocation(linearOpMode);
    }
    

    public void HALT() {
        frontRight.stopMotor();
        backRight.stopMotor();
        frontLeft.stopMotor();
        backLeft.stopMotor();
    }

    public boolean isMoving(){
        return frontRight.motor.isBusy() || frontLeft.motor.isBusy() || backRight.motor.isBusy() || backLeft.motor.isBusy();
    }

    public void resetEncoders() {
        // Fix all the directions of the motors.
        frontRight.setInverted(true);
//        frontLeft.setInverted(true);
        backRight.setInverted(true);
//        backLeft.setInverted(true);
        frontRight.resetEncoder();
        frontLeft.resetEncoder();
        backRight.resetEncoder();
        backLeft.resetEncoder();
        runUsingEncoders();
    }

    public void runUsingEncoders(){
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
    }

    public void setDriveTargetPos(double frpos, double flpos, double brpos, double blpos) {
        frontRight.setTargetPosition((int) frpos);
        frontLeft.setTargetPosition((int) flpos);
        backRight.setTargetPosition((int) brpos);
        backLeft.setTargetPosition((int) blpos);
    }

    public void setDrivePower(double frPower, double flPower, double brPower, double blPower) {
        runOnPower();
        frontRight.set(frPower);
        frontLeft.set(flPower);
        backRight.set(brPower);
        backLeft.set(blPower);
    }

    public boolean overCargo(){
//        while(linearOpMode.opModeIsActive()) {
//            Log.i("Distance from the ground(OverCargo): ", String.valueOf(getIMUAngle(Axis.Y)));
//        }
        if(getIMUAngle(Axis.Y) > 3){
            Log.i("Over Cargo: ","true");
        }
        else{
            Log.i("Over Cargo: ","false");
        }
        return getIMUAngle(Axis.Y) > 3;
    }

    public void setAllDriveTargetPos(double pos){
        setDriveTargetPos(pos,pos,pos,pos);
    }

    public void setAllDrivePower(double power) {
        setDrivePower(power, power, power, power);
    }

    public void strafe(Direction dir, double power, double targetDistance) {
        targetRotations = targetDistance / wheelCircumference;
        targetTicks = targetRotations * driveTicksPerRev;
        targetAngle = getIMUAngle(Axis.X);
        resetEncoders();
        setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);

        switch (dir) {
            case RIGHT:
                brPower  = power;
                frPower = flPower = blPower = power;
                setDriveTargetPos(-targetTicks, targetTicks, targetTicks, -targetTicks);
                break;
            case LEFT:
                flPower = brPower = power;
                frPower = blPower = power;
                setDriveTargetPos(targetTicks, -targetTicks, -targetTicks, targetTicks);
                break;
        }

        while((!frontRight.atTargetPosition() || !frontLeft.atTargetPosition() || !backRight.atTargetPosition() || !backLeft.atTargetPosition()) && linearOpMode.opModeIsActive()) {
            if(frontRight.atTargetPosition()){
                frPower = 0;
            }
            if(frontLeft.atTargetPosition()){
                flPower = 0;
            }
            if(backRight.atTargetPosition()){
                brPower = 0;
            }
            if(backLeft.atTargetPosition()){
                blPower = 0;
            }
            telemetry.addData("target: ",targetTicks);
            telemetry.addData("frticks: ",frontRight.getCurrentPosition());
            telemetry.addData("flticks: ",frontLeft.getCurrentPosition());
            telemetry.addData("brticks: ",backRight.getCurrentPosition());
            telemetry.addData("blticks: ",backLeft.getCurrentPosition());
            telemetry.update();
            setDrivePower(frPower, flPower, brPower, blPower);
        }
        HALT();
    }

    // Tolerance in ticks
    public void setDriveTolerance(double frtolerance, double fltolerance, double brtolerance, double bltolerance) {
        frontRight.setPositionTolerance(frtolerance);
        frontLeft.setPositionTolerance(fltolerance);
        backRight.setPositionTolerance(brtolerance);
        backLeft.setPositionTolerance(bltolerance);
    }

    public void drive(Direction dir, double power, double targetDistance) {
        Log.i("Autonomous", "Drive called. " + dir.name());
        targetAngle = getIMUAngle(Axis.X);
        targetRotations = targetDistance / wheelCircumference;
        targetTicks = targetRotations * driveTicksPerRev;
        resetEncoders();
        setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
        switch (dir) {
            case FORWARDS:
                setDriveTargetPos(targetTicks, targetTicks, targetTicks, targetTicks);
                while (((frCurrentTicks < targetTicks && flCurrentTicks < targetTicks) || (brCurrentTicks < targetTicks && blCurrentTicks < targetTicks)) && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.X);
                    correction = (targetAngle - currentAngle) * driveGain;
                    cappedPower = Range.clip(power + correction, -1, 1);
                    correctedCappedPower = Range.clip(power - correction, -1, 1);
                    frPower = brPower = cappedPower;
                    flPower = blPower = correctedCappedPower;
                    frCurrentTicks = frontRight.getCurrentPosition();
                    flCurrentTicks = frontLeft.getCurrentPosition();
                    brCurrentTicks = backRight.getCurrentPosition();
                    blCurrentTicks = backLeft.getCurrentPosition();
                    if (frCurrentTicks >= targetTicks) {
                        frPower = 0;
                    }
                    if (flCurrentTicks >= targetTicks) {
                        flPower = 0;
                    }
                    if (brCurrentTicks >= targetTicks) {
                        brPower = 0;
                    }
                    if (blCurrentTicks >= targetTicks) {
                        blPower = 0;
                    }
                    runOnPower();
                    setDrivePower(frPower, flPower, brPower, blPower);
                    telemetry.addData("Correction: ", correction);
                    telemetry.addData("Current Angle: ", currentAngle);
                    telemetry.addData("Current Power: ", "FR: " + correctedCappedPower + " FL: " + cappedPower + " BR: " + correctedCappedPower + " BL: " + cappedPower);
                    telemetry.addData("Current Ticks: ", "FR: " + frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
                    telemetry.addData("Target Ticks: ", (int) targetTicks);
                    telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
                    telemetry.update();
                }
                break;
            case BACKWARDS:
                setDriveTargetPos(-targetTicks, -targetTicks, -targetTicks, -targetTicks);
                while (((frCurrentTicks > -targetTicks && flCurrentTicks > -targetTicks) || (brCurrentTicks > -targetTicks && blCurrentTicks > -targetTicks)) && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.X);
                    correction = (targetAngle - currentAngle) * driveGain;
                    cappedPower = -Range.clip(power - correction, -1, 1);
                    correctedCappedPower = -Range.clip(power + correction, -1, 1);
                    frPower = brPower = cappedPower;
                    flPower = blPower = correctedCappedPower;
                    frCurrentTicks = frontRight.getCurrentPosition();
                    flCurrentTicks = frontLeft.getCurrentPosition();
                    brCurrentTicks = backRight.getCurrentPosition();
                    blCurrentTicks = backLeft.getCurrentPosition();
                    if (frCurrentTicks >= targetTicks) {
                        frPower = 0;
                    }
                    if (flCurrentTicks >= targetTicks) {
                        flPower = 0;
                    }
                    if (brCurrentTicks >= targetTicks) {
                        brPower = 0;
                    }
                    if (blCurrentTicks >= targetTicks) {
                        blPower = 0;
                    }
                    runOnPower();
                    setDrivePower(frPower, flPower, brPower, blPower);
                    telemetry.addData("Correction: ", correction);
                    telemetry.addData("Current Angle: ", currentAngle);
                    telemetry.addData("Current Power: ", "FR: " + correctedCappedPower + " FL: " + cappedPower + " BR: " + correctedCappedPower + " BL: " + cappedPower);
                    telemetry.addData("Current Ticks: ", "FR: " + frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
                    telemetry.addData("Target Ticks: ", (int) targetTicks);
                    telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
                    telemetry.update();
                }
                break;
        }
        this.turn(power, targetAngle);
//        finalCorrection(frontRight.getCurrentPosition(), frontLeft.getCurrentPosition(), backRight.getCurrentPosition(), backLeft.getCurrentPosition(), power);
        telemetry.addData("Robot has moved: ", targetDistance + "cm " + dir);
        telemetry.update();
        HALT();
    }

    public void runOnPower(){
        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
    }

    //. GOD function!!!
    public void turn(double power, double degrees) {
        Log.i("Autonomous", "Turn called. " + degrees + "deg");
        runOnPower();
        double tolerance = 4;
        targetAngle = degrees;
        if(targetAngle > 180) targetAngle -= 360;
        double rangeMin = targetAngle-tolerance/2;
        double rangeMax = targetAngle+tolerance/2;

        do {
            currentAngle = getIMUAngle(Axis.X);
            if(targetAngle - currentAngle > 0) setDrivePower(power, -power, power, -power);
            else setDrivePower(-power, power, -power, power);
            telemetry.addData("Target Angle: ",targetAngle);
            telemetry.addData("Current Angle: ",currentAngle);
            telemetry.addData("ranges: ", rangeMin + " " + rangeMax);
            telemetry.update();
        }
        while (!(rangeMin < currentAngle && rangeMax > currentAngle));
//        this.finalTurn(0.1,degrees);
        resetEncoders();
        HALT();
    }
    
       public void finalTurn(double power, double degrees) {

        Log.i("Autonomous", "Turn called. " + degrees + "deg");

        runOnPower();
        double tolerance = 0;
        targetAngle = degrees;
        if(targetAngle > 180) targetAngle -= 360;
        double rangeMin = targetAngle-tolerance/2;
        double rangeMax = targetAngle+tolerance/2;

        do {
            currentAngle = getIMUAngle(Axis.X);
            if(targetAngle - currentAngle > 0) setDrivePower(power, -power, power, -power);
            else setDrivePower(-power, power, -power, power);
            telemetry.addData("Target Angle: ",targetAngle);
            telemetry.addData("Current Angle: ",currentAngle);
            telemetry.addData("ranges: ", rangeMin + " " + rangeMax);
            telemetry.update();
        } while (!(rangeMin < currentAngle && rangeMax > currentAngle));
        resetEncoders();
        HALT();
    }

    public void turn(double power, double degrees, boolean oneSide) {
        runOnPower();
        double tolerance = 6;
        targetAngle = degrees;
        if(targetAngle > 180) targetAngle = 360 - targetAngle;
        double rangeMin = targetAngle-tolerance/2;
        double rangeMax = targetAngle+tolerance/2;

        do {
            currentAngle = getIMUAngle(Axis.X);
            if(targetAngle - currentAngle > 0) setDrivePower(power, (oneSide ? 0 : -power), power, (oneSide ? 0 : -power));
            else setDrivePower((oneSide ? 0 : -power), power, (oneSide ? 0 : -power), power);
            telemetry.addData("Target Angle: ",targetAngle);
            telemetry.addData("Current Angle: ",currentAngle);
            telemetry.addData("ranges: ", rangeMin + " " + rangeMax);
            telemetry.update();
        } while (!(rangeMin < currentAngle && rangeMax > currentAngle));
        HALT();
        resetEncoders();
    }

    public void moveArm(int pos, double power) {
        power = 0.08;
        arm.setTargetPosition(pos);
        armPower = power;
        while (!arm.atTargetPosition() && linearOpMode.opModeIsActive()) {}
        this.pause(100);
    }

    public void intake(Direction dir, double power) {
        double startTime = System.currentTimeMillis();
        while(linearOpMode.opModeIsActive()) {
            boolean canStop = System.currentTimeMillis() > startTime+1500;
            telemetry.addData("Cargo: ", cargoDetection());
            telemetry.update();

            if(dir.equals(Direction.IN)) {
                collector.set(power);

                telemetry.addData("Cargo:", cargoDetection());
                telemetry.update();

                if(!cargoDetection().equals("None") && canStop) {
                    break;
                }
            } else if(dir.equals(Direction.OUT)) {
                collector.set(-power);

                if (cargoDetection().equals("None") && canStop) {
                    break;
                }
            }
        }
        collector.set(0);
    }

    public void pause(long time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException ignored) {}
    }

    public void duckSpin(double power, long timeToSpin) {
//        duckSpinner.setRunMode(Motor.RunMode.PositionControl);
        //duckSpinnersStartPos = duckSpinner.getCurrentPosition();
        long timeMillis = System.currentTimeMillis();
        while(System.currentTimeMillis() < timeMillis+timeToSpin && linearOpMode.opModeIsActive()) {
//            if (power > 0) {
//                if (duckSpinnersStartPos < duckSpinner.getCurrentPosition()) {
//                    power += 0.03;
//                }
//            }
//            else if (power < 0) {
//                if (duckSpinnersStartPos < duckSpinner.getCurrentPosition()) {
//                    power -= 0.03;
//                }
//            }
//            duckSpinner.setRunMode(Motor.RunMode.RawPower);
            duckSpinner.set(power);
            telemetry.addData("Duck Spinner Power: ", String.valueOf(power));
            telemetry.update();
        }
        duckSpinner.setRunMode(Motor.RunMode.RawPower);
        duckSpinner.set(0);
    }

    public double getVelocity(){
        double frVelo = frontRight.getCorrectedVelocity();
        double flVelo = frontLeft.getCorrectedVelocity();
        double brVelo = backRight.getCorrectedVelocity();
        double blVelo = backLeft.getCorrectedVelocity();
        double averageVelo = (frVelo + flVelo + brVelo + blVelo)/4;
//        telemetry.addData("Current Velocity: ",averageVelo);
//        telemetry.update();
        return averageVelo;
    }

    public void driverOverBarriers(Direction dir,double power){
        double currentDistance = frontDistance.getDistance(DistanceUnit.CM);
        Log.i("CurrentDistance: ", String.valueOf(currentDistance));
        switch(dir) {
            case FORWARDS:
                while (currentDistance > afterBarriers) {
                    currentDistance = frontDistance.getDistance(DistanceUnit.CM);
                    Log.i("Forwards Current Distance: ", String.valueOf(currentDistance));
                    this.setDrivePower(power, power, power, power);
                }
                break;
            case BACKWARDS:
                while (currentDistance < beforeBarriers) {
                    currentDistance = frontDistance.getDistance(DistanceUnit.CM);
                    Log.i("Backwards Current Distance: ", String.valueOf(frontDistance.getDistance(DistanceUnit.CM)));
//                    this.drive(dir, power,30);
                    this.setDrivePower(-power, -power, -power, -power);
                }
                break;
        }
        this.HALT();
        Log.i("CurrentDistance: ", String.valueOf(frontDistance.getDistance(DistanceUnit.CM)));
    }

    private boolean isInRange(float[] HSV, Scalar low, Scalar high) {
        double lowH = low.val[0];
        double lowS = low.val[1];
        double lowV = low.val[2];

        double highH = high.val[0];
        double highS = high.val[1];
        double highV = high.val[2];

        boolean H = lowH < HSV[0] && HSV[0] > highH;
        boolean S = lowS < HSV[1] && HSV[1] > highS;
        boolean V = lowV < HSV[2] && HSV[2] > highV;

        return H && S && V;
    }

    private float[] rgbToHSV(int r, int g, int b) {
        r /= 255;
        g /= 255;
        b /= 255;

        float h = 0;
        float s = 0;
        float v = 0;

        float cmax = Math.max(r, Math.max(g, b));
        float cmin = Math.min(r, Math.min(g, b));
        float diff = cmax - cmin;

        if(cmax == cmin) {
            h = 0;
        }
        else if(cmax == r) {
            h = (60 * ((g-b) / diff) + 360) % 360;
        }
        else if(cmax == g) {
            h = (60 * ((b-r) / diff) + 120) % 120;
        }
        else if(cmax == b) {
            h = (60 * ((r-g) / diff) + 240) % 240;
        }

        if(cmax == 0) {
            s = 0;
        } else {
            s = diff / cmax * 100;
        }

        v = cmax * 100;

        return new float[]{h, s, v};
    }

    public String cargoDetection(){
        // Cargo detection
        int red = cargoDetector.red();
        int green = cargoDetector.green();
        int blue = cargoDetector.blue();
        float[] hsvValues = {0x0F, 0x0F, 0x0F};
        cargoDetector.RGBtoHSV(red, green, blue, hsvValues);
        Scalar lowCubeHSV = new Scalar(50, 100, 0);
        Scalar highCubeHSV = new Scalar(70, 255, 255);
        Scalar lowBallHSV = new Scalar(0, 0, 0);
        Scalar highBallHSV = new Scalar(0, 0, 255);

        if(isInRange(hsvValues, lowCubeHSV, highCubeHSV)) {
            return "Cube OR Duck";
        }
        else if(isInRange(hsvValues, lowBallHSV, highBallHSV)) {
            return "Ball";
        }
        else return "None";

//        Log.i("Color 0: ", String.valueOf(color[0]));
//        Log.i("Color 1: ", String.valueOf(color[1]));
//        Log.i("Color 2: ", String.valueOf(color[2]));
//        if (color[0] > 34 && color[0] < 36 && 0 < color[1] && color[1] < 0.45 && 0.9 < color[2] && color[2] < 1.3) {
//            return "Ball";
//        }
//        else if(color[0] > 32 && color[0] < 35 && 0.45 < color[1] && color[1] < 1 && 0.9 < color[2] && color[2] < 1.5) {
//            return "Cube OR Duck";
//        }
//        else {
//            return "None";
//        }
    }

    // Class constructor.
    // Important initialization code. Modify only if needed.
    public Robot(List motors, LinearOpMode linearOpMode) {
        hardwareMap = linearOpMode.hardwareMap;

        detector = new TseDetector();
        int cameraMonitorViewId = linearOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        this.linearOpMode = linearOpMode;
        backLeft = (Motor)motors.get(0);
        frontLeft = (Motor)motors.get(1);
        backRight = (Motor)motors.get(2);
        frontRight = (Motor)motors.get(3);
        arm = (Motor) motors.get(4);
        collector = (Motor) motors.get(5);
        duckSpinner = (Motor) motors.get(6);
        imu = (RevIMU) motors.get(7);
        imu.init();
        cargoDetector = (SensorColor) motors.get(8);
        frontDistance = (SensorRevTOFDistance) motors.get(9);

        // Set the zero power behavior of the motors.
        // We don't want them to slide after every trajectory or else we will lose accuracy.
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//        duckSpinner.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Run using encoders!!!
        arm.setRunMode(Motor.RunMode.PositionControl);
        resetEncoders();

        this.telemetry = new MultipleTelemetry(linearOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        this.armThread = new Thread(() -> {
            linearOpMode.waitForStart();
            arm.resetEncoder();
            arm.setRunMode(Motor.RunMode.PositionControl);
            while(linearOpMode.opModeIsActive()) {
                arm.setPositionTolerance(40);
                if(!arm.atTargetPosition()) arm.set(armPower);
                else {
                    arm.set(0);
                }
            }
            arm.set(0);
        });
        this.armThread.start();

        // Initialize the new detector
//        Detector detector = new Detector(hardwareMap);
    }
}