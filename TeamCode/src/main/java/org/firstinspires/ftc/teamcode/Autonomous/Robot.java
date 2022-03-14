package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.visionv1.*;

// Item detector based on the ground tape
import org.firstinspires.ftc.teamcode.Autonomous.visionv2.Detector;
import org.firstinspires.ftc.teamcode.events.EventDispatcher;
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
    MotorGroup duckSpinners;
    RevIMU imu;
    SensorRevTOFDistance cargoDetector;
    double armPower = 0;
    boolean armHoldPosition = false;
//    TouchSensor touchSensorSideRight;
//    TouchSensor touchSensorSideLeft;
//    TouchSensor touchSensorFrontLeft;
    
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
    // Old Wheels
    // public static double wheelRadius = 7/2;
    // New wheels
    public static double wheelRadius = 15/2;
    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
    public static double centerToWheel = 21;
    public static double turnCircumference = 2 * Math.PI * centerToWheel;

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
    public double driveErrorTolerance = 20; // in ticks
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

    // cargoDetector
    public String detectedCargo = "None";
    public double cubeHeight= 5.08;
    public double ballHeight = 6.99;
    public double duckHeight = 5.4;
    public double currentCargoDistance = 0;
    public double collectorBoxHeight = 15;

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
        HIGH(-1800),
        DOWN(0);

        public final int label;
        private Position(int label) {
            this.label = label;
        }
    }

    private boolean atTargetPosition(Motor motor, double target) {
        if(target < 0) target *= -1;
        if(motor.getCurrentPosition() >= target) return true;
        return false;
    }

//    public String inContact() {
//        touchSensorSideRight = hardwareMap.get(TouchSensor.class, "rightSideTouch");
//        touchSensorSideLeft = hardwareMap.get(TouchSensor.class, "leftSideTouch");
//        touchSensorFrontLeft = hardwareMap.get(TouchSensor.class, "frontLeftTouch");
//
//        if (touchSensorSideLeft.isPressed()) {
//            return "Left side";
//        } else if (touchSensorSideRight.isPressed()) {
//            return "Right side";
//        } else if (touchSensorFrontLeft.isPressed()){
//            return "Front side";
//        } else {
//            return "Not In Contact";
//        }
//    }
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
//        detector = new TseDetector();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(detector);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(webcam, 0);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });

        TseDetector.Location location = detector.getLocation(linearOpMode);
        return location;
    }

//    public Detector.ElementPosition getTsePos() {
//        Detector.ElementPosition pos = new Detector(hardwareMap).getElementPosition();
//        telemetry.addData("The shipping element is located at the ", pos);
//        telemetry.update();
//        return pos;
//    }

    private void HALT() {
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
//        frontLeft.setInverted(true);
//        backLeft.setInverted(true);
        // frontRight.setInverted(true);
        backRight.setInverted(true);
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
        frontRight.set(frPower);
        frontLeft.set(flPower);
        backRight.set(brPower);
        backLeft.set(blPower);
    }

//    public void strafe(Direction dir, double power, double targetDistance) {
//        targetRotations = targetDistance / wheelCircumference;
//        targetTicks = targetRotations * driveTicksPerRev;
//        targetAngle = getIMUAngle(Axis.X);
//        resetEncoders();
//        setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
//        switch (dir) {
//            case LEFT:
//                setDriveTargetPos(targetTicks, -targetTicks, -targetTicks, targetTicks);
//                while ((frCurrentTicks <= targetTicks || flCurrentTicks <= targetTicks || brCurrentTicks <= targetTicks || blCurrentTicks <= targetTicks) && linearOpMode.opModeIsActive()) {
//                    currentAngle = getIMUAngle(Axis.X);
//                    correction = (targetAngle - currentAngle) * strafeGain;
//                    cappedPower = Range.clip(power, -1, 1);
//                    correctedCappedPower = Range.clip(power - correction, -1, 1);
//                    frPower = brPower = correctedCappedPower;
//                    flPower = blPower = cappedPower;
//                    frCurrentTicks = frontRight.getCurrentPosition();
//                    flCurrentTicks = frontLeft.getCurrentPosition();
//                    brCurrentTicks = backRight.getCurrentPosition();
//                    blCurrentTicks = backLeft.getCurrentPosition();
//                    if (frCurrentTicks >= targetTicks) {
//                        frPower = 0;
//                    }
//                    if (flCurrentTicks >= targetTicks) {
//                        flPower = 0;
//                    }
//                    if (brCurrentTicks >= targetTicks) {
//                        brPower = 0;
//                    }
//                    if (blCurrentTicks >= targetTicks) {
//                        blPower = 0;
//                    }
//                    setDrivePower(-frPower, flPower, brPower, -blPower);
//                    telemetry.addData("Correction: ", correction);
//                    telemetry.addData("Current Angle: ", currentAngle);
//                    telemetry.addData("Current Power: ", "FR: " + frPower + " FL: " + flPower + " BR: " + brPower + " BL: " + blPower);
//                    telemetry.addData("Current Ticks: ", "FR: "+ frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
//                    telemetry.addData("Target Ticks: ", (int) targetTicks);
//                    telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
//                    telemetry.update();
//                }
//                break;
//            case RIGHT:
//                setDriveTargetPos(-targetTicks, targetTicks, -targetTicks, targetTicks);
//                while ((frCurrentTicks <= targetTicks || flCurrentTicks <= targetTicks || brCurrentTicks <= targetTicks || blCurrentTicks <= targetTicks) && linearOpMode.opModeIsActive()) {
//                    currentAngle = getIMUAngle(Axis.X);
//                    correction = (targetAngle - currentAngle) * strafeGain;
//                    cappedPower = -Range.clip(power, -1, 1);
//                    correctedCappedPower = -Range.clip(power - correction, -1, 1);
//                    frPower = brPower = correctedCappedPower;
//                    flPower = blPower = cappedPower;
//                    frCurrentTicks = frontRight.getCurrentPosition();
//                    flCurrentTicks = frontLeft.getCurrentPosition();
//                    brCurrentTicks = backRight.getCurrentPosition();
//                    blCurrentTicks = backLeft.getCurrentPosition();
//                    if (frCurrentTicks >= targetTicks) {
//                        frPower = 0;
//                    }
//                    if (flCurrentTicks >= targetTicks) {
//                        flPower = 0;
//                    }
//                    if (brCurrentTicks >= targetTicks) {
//                        brPower = 0;
//                    }
//                    if (blCurrentTicks >= targetTicks) {
//                        blPower = 0;
//                    }
//                    setDrivePower(frPower, -flPower, -brPower, blPower);
//                    telemetry.addData("Correction: ", correction);
//                    telemetry.addData("Current Angle: ", currentAngle);
//                    telemetry.addData("Current Power: ", correctedCappedPower + " " + cappedPower + " " + -correctedCappedPower + " " + -cappedPower);
//                    telemetry.addData("Current Ticks: ", currentTicks);
//                    telemetry.addData("Target Ticks: ", (int) targetTicks);
//                    telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance);
//                    telemetry.update();
//                    break;
//                }
//                HALT();
//                telemetry.addData("Robot has strafed: ", String.valueOf(targetDistance), " ", dir);
//                telemetry.update();
//        }
//    }

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
            if(frontRight.getCurrentPosition() >= targetTicks){
                frPower = 0;
            }
            if(frontLeft.atTargetPosition()){
                flPower = 0;
            }
            if(backRight.getCurrentPosition() >= targetTicks){
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

//    public void turn(Direction dir, double power, double degrees) {
//        frPower = flPower = brPower = blPower = power;
//        targetRotations = degrees / 360 * turnCircumference;
//        targetTicks = targetRotations * driveTicksPerRev;
//        resetEncoders();
//        // setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
//        setDriveTargetPos(ticksToTurn, ticksToTurn, ticksToTurn, ticksToTurn);
//        switch (dir){
//            case LEFT:
//                while (((frCurrentTicks < targetTicks && flCurrentTicks < targetTicks) || (brCurrentTicks < targetTicks && blCurrentTicks < targetTicks)) && linearOpMode.opModeIsActive()) {
//                    if (frCurrentTicks >= targetTicks) {
//                        frPower = 0;
//                    }
//                    if (flCurrentTicks >= targetTicks) {
//                        flPower = 0;
//                    }
//                    if (brCurrentTicks >= targetTicks) {
//                        brPower = 0;
//                    }
//                    if (blCurrentTicks >= targetTicks) {
//                        blPower = 0;
//                    }
//                    setDrivePower(-frPower, -flPower, brPower, blPower);
//                }
//                break;
//            case RIGHT:
//                while (((frCurrentTicks < targetTicks && flCurrentTicks < targetTicks) || (brCurrentTicks < targetTicks && blCurrentTicks < targetTicks)) && linearOpMode.opModeIsActive()) {
//                    if (frCurrentTicks >= targetTicks) {
//                        frPower = 0;
//                    }
//                    if (flCurrentTicks >= targetTicks) {
//                        flPower = 0;
//                    }
//                    if (brCurrentTicks >= targetTicks) {
//                        brPower = 0;
//                    }
//                    if (blCurrentTicks >= targetTicks) {
//                        blPower = 0;
//                    }
//                    setDrivePower(frPower, flPower, -brPower, -blPower);
//                }
//                break;
//            }
//        while (isMoving() && linearOpMode.opModeIsActive()) {
//            telemetry.addData("Robot is turning ", String.valueOf(degrees), "to the ", dir, "with a power of ", power);
//            telemetry.addData("Target Ticks to turn: ", ticksToTurn);
//            telemetry.update();
//        }
//        telemetry.addData("Robot has turned", String.valueOf(degrees), "to the", dir);
//        telemetry.update();
//    }



//    public void moveArm(int pos, double power) {
//        //TODO: Calibrate the ticks needed for each of the 3 levels
//        arm.setRunMode(Motor.RunMode.PositionControl);
//        arm.resetEncoder();
//        arm.setPositionTolerance(40);
//        lastClawPosition = pos;
//        arm.setTargetPosition(pos);
////        if(arm.getCurrentPosition() > lastClawPosition){
////            power = -power;
////        }
//        while (!arm.atTargetPosition()) {
//            armPower = power;
//            telemetry.addData("The arm is moving to ", lastClawPosition +" with a power of " + power);
//            telemetry.addData("Arm Ticks: ", arm.getCurrentPosition());
//            telemetry.update();
//            arm.setRunMode(Motor.RunMode.PositionControl);
//        }
//        telemetry.addData("The arm has moved to the ", String.valueOf(pos), " from ", lastClawPosition);
//        telemetry.update();
//    }

    public void moveArm(int pos, double power) {
        arm.setTargetPosition(pos);
        armPower = power;
        while (!arm.atTargetPosition()) {}
        try {
            Thread.sleep(100);
        } catch (Exception ignored) {}
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
//        try {
//            Thread.sleep(500);
//        } catch (InterruptedException ignored) {}
    }

    public void duckSpin(double power, long timeToSpin) {
        duckSpinners.setRunMode(Motor.RunMode.PositionControl);
        duckSpinnersStartPos = duckSpinners.getCurrentPosition;
        long timeMillis = System.currentTimeMillis();
        while(System.currentTimeMillis() < timeMillis+timeToSpin && linearOpMode.opModeIsActive()) {
            if(duckSpinners.getCurrentPosition){
              if(power > 0){
                power += 0.03;
              }
              else if (power < 0){
                power -= 0.03;
              }
            duckSpinners.set(power);
            }
        duckSpinners.set(0);
    }

    public String cargoDetection(){
        // Cargo detection
        // The less the distance from the ground subtraction the higher object we are possessing
        currentCargoDistance = cargoDetector.getDistance(DistanceUnit.CM);
        if (6.5 < collectorBoxHeight - currentCargoDistance && collectorBoxHeight - currentCargoDistance < 9) {
            return "Ball";
        }
        else if(3.5 < collectorBoxHeight - currentCargoDistance && collectorBoxHeight - currentCargoDistance < 6) {
            return "Cube OR Duck";
        }
        else {
            return "None";
        }
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
        duckSpinners = (MotorGroup) motors.get(6);
        imu = (RevIMU) motors.get(7);
        imu.init();
        cargoDetector = (SensorRevTOFDistance) motors.get(8);

        // Set the zero power behavior of the motors.
        // We don't want them to slide after every trajectory or else we will lose accuracy.
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

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