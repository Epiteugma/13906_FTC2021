package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.visionv1.*;

// Item detector based on the ground tape
import org.firstinspires.ftc.teamcode.Autonomous.visionv2.Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.List;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class Robot {
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
    public double collectorBoxHeight;


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
        LOW,
        MID,
        HIGH,
        DOWN
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
        TseDetector detector;
        detector = new TseDetector();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        TseDetector.Location location = detector.getLocation();
        return location;
    }

//    public Detector.ElementPosition getTsePos() {
//        Detector.ElementPosition pos = new Detector(hardwareMap).getElementPosition();
//        linearOpMode.telemetry.addData("The shipping element is located at the ", pos);
//        linearOpMode.telemetry.update();
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

    public void strafe(Direction dir, double power, double targetDistance) {
        targetRotations = targetDistance / wheelCircumference;
        targetTicks = targetRotations * driveTicksPerRev;
        resetEncoders();
        setDriveTargetPos(targetTicks, targetTicks, targetTicks, targetTicks);
        // setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
        switch (dir) {
            case LEFT:
                while (frCurrentTicks >= targetTicks || flCurrentTicks >= targetTicks || brCurrentTicks >= targetTicks || blCurrentTicks >= targetTicks && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.X);
                    correction = (targetAngle - currentAngle) * strafeGain;
                    cappedPower = Range.clip(power, -1, 1);
                    correctedCappedPower = Range.clip(power - correction, -1, 1);
                    frPower = brPower = correctedCappedPower;
                    flPower = blPower = cappedPower;
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
                    setDrivePower(-frPower, -flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", "FR: " + frPower + " FL: " + flPower + " BR: " + brPower + " BL: " + blPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", "FR: "+ frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", (int) targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
                    linearOpMode.telemetry.update();
                }
            case RIGHT:
                while (frCurrentTicks >= targetTicks || flCurrentTicks >= targetTicks || brCurrentTicks >= targetTicks || blCurrentTicks >= targetTicks && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.X);
                    correction = (targetAngle - currentAngle) * strafeGain;
                    cappedPower = -Range.clip(power, -1, 1);
                    correctedCappedPower = -Range.clip(power - correction, -1, 1);
                    frPower = brPower = correctedCappedPower;
                    flPower = blPower = cappedPower;
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
                    setDrivePower(frPower, flPower, -brPower, -blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", correctedCappedPower + " " + cappedPower + " " + -correctedCappedPower + " " + -cappedPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", currentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", (int) targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance);
                    linearOpMode.telemetry.update();
                }
                linearOpMode.telemetry.addData("Robot has moved: ", String.valueOf(targetDistance), " ", dir);
                linearOpMode.telemetry.update();
        }
    }

    // Tolerance in ticks
    public void setDriveTolerance(double frtolerance, double fltolerance, double brtolerance, double bltolerance) {
        frontRight.setPositionTolerance(frtolerance);
        frontLeft.setPositionTolerance(fltolerance);
        backRight.setPositionTolerance(brtolerance);
        backLeft.setPositionTolerance(bltolerance);
    }

    public void drive(Direction dir, double power, double targetDistance) {
        targetAngle = getIMUAngle(Axis.X);
        targetRotations = targetDistance / wheelCircumference;
        targetTicks = targetRotations * driveTicksPerRev;
        resetEncoders();
        // setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
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
                    setDrivePower(frPower, flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", "FR: " + correctedCappedPower + " FL: " + cappedPower + " BR: " + correctedCappedPower + " BL: " + cappedPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", "FR: " + frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", (int) targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
                    linearOpMode.telemetry.update();
                }
                break;
            case BACKWARDS:
                setDriveTargetPos(-targetTicks, -targetTicks, -targetTicks, -targetTicks);
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
                    setDrivePower(frPower, flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", "FR: " + correctedCappedPower + " FL: " + cappedPower + " BR: " + correctedCappedPower + " BL: " + cappedPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", "FR: " + frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", (int) targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
                    linearOpMode.telemetry.update();
                }
                break;
        }
        finalCorrection(frontRight.getCurrentPosition(), frontLeft.getCurrentPosition(), backRight.getCurrentPosition(), backLeft.getCurrentPosition(), power);

//        while (isMoving() && linearOpMode.opModeIsActive()) {
//            linearOpMode.telemetry.addData("Robot is moving" ," (when it shouldn't)");
//            linearOpMode.telemetry.update();
//        }

        linearOpMode.telemetry.addData("Robot has moved: ", targetDistance + "cm " + dir);
        linearOpMode.telemetry.update();
    }

    public void turn(Direction dir, double power, double degrees) {
        frPower = flPower = brPower = blPower = power;
        targetRotations = degrees / 360 * turnCircumference;
        targetTicks = targetRotations * driveTicksPerRev;
        resetEncoders();
        // setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
        setDriveTargetPos(ticksToTurn, ticksToTurn, ticksToTurn, ticksToTurn);
        switch (dir){
            case LEFT:
                while (((frCurrentTicks < targetTicks && flCurrentTicks < targetTicks) || (brCurrentTicks < targetTicks && blCurrentTicks < targetTicks)) && linearOpMode.opModeIsActive()) {
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
                    setDrivePower(-frPower, -flPower, brPower, blPower);
                }
                break;
            case RIGHT:
                while (((frCurrentTicks < targetTicks && flCurrentTicks < targetTicks) || (brCurrentTicks < targetTicks && blCurrentTicks < targetTicks)) && linearOpMode.opModeIsActive()) {
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
                    setDrivePower(frPower, flPower, -brPower, -blPower);
                }
                break;
            }
        while (isMoving() && linearOpMode.opModeIsActive()) {
            linearOpMode.telemetry.addData("Robot is turning ", String.valueOf(degrees), "to the ", dir, "with a power of ", power);
            linearOpMode.telemetry.addData("Target Ticks to turn: ", ticksToTurn);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("Robot has turned", String.valueOf(degrees), "to the", dir);
        linearOpMode.telemetry.update();
    }

    public void moveArm(Position pos, double power) {
        //TODO: Calibrate the ticks needed for each of the 3 levels
        arm.resetEncoder();
        int lowPosition = -370;
        int midPosition = -1000;
        int highPosition = -1800;
        arm.setPositionTolerance(40);
        switch(pos) {
            case LOW:
//                if (lowPosition > lastClawPosition) {
//                    lowPosition = lowPosition + lastClawPosition;
//                }
//                else {
//                    lowPosition = lowPosition - lastClawPosition;
//                }
                arm.setTargetPosition(lowPosition);
                lastClawPosition = lowPosition;
                break;
            case MID:
//                if (midPosition > lastClawPosition) {
//                    midPosition = midPosition + lastClawPosition;
//                }
//                else {
//                    midPosition = midPosition - lastClawPosition;
//                }
                arm.setTargetPosition(midPosition);
                lastClawPosition = midPosition;
                break;
            case HIGH:
//                // No need to check if above it will never be above the highest possible position
//                highPosition = highPosition - lastClawPosition;
                arm.setTargetPosition(highPosition);
                lastClawPosition = highPosition;
                break;
            case DOWN:
                arm.setTargetPosition(-lastClawPosition);
                break;
        }
        if (!collector.atTargetPosition()) {
            arm.set(power);
            linearOpMode.telemetry.addData("The arm is moving to the ", String.valueOf(pos), " from ", lastClawPosition, " with a power of ", power);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("The arm has moved to the ", String.valueOf(pos), " from ", lastClawPosition);
        linearOpMode.telemetry.update();
    }

    public void intake(Direction dir, double power) {
        switch(dir){
            case IN:
                if(cargoDetection().equals("None")){
                    linearOpMode.telemetry.addData("We are still NOT in possession of cargo...", "None");
                    linearOpMode.telemetry.update();
                    collector.set(power);
                }
                break;
            case OUT:
                if(!cargoDetection().equals("None")){
                    linearOpMode.telemetry.addData("We are still in possession of cargo: ", "(" + cargoDetection() + ")");
                    linearOpMode.telemetry.update();
                    collector.set(-power);
                }
                break;
            }
        collector.stopMotor();
        }

    public void duckSpin(double power, long timeToSpin) {
        // TODO: calibrate time to spin.
        long timeMillis = 0;
        while(System.currentTimeMillis()+timeToSpin > timeMillis && linearOpMode.opModeIsActive()) {
            duckSpinners.set(power);
        }
        duckSpinners.stopMotor();
    }

    // TODO: Complete this function next meeting
    public void finalCorrection(int frTicks, int flTicks, int brTicks, int blTicks, double power) {
        int frontCorrection = Math.abs(frTicks) - Math.abs(flTicks);
        int backCorrection = Math.abs(brTicks) - Math.abs(blTicks);
        int frontTarget;
        int backTarget;
        double dirPower = (frontCorrection + backCorrection) / 2 < 0 ? -power : power;
        if(frTicks < 0) {
            frontTarget = frTicks - frontCorrection;
            backTarget = brTicks - backCorrection;
            while (frontRight.getCurrentPosition() > frontTarget && backRight.getCurrentPosition() > backTarget) {
                setDrivePower(dirPower, 0, dirPower, 0);
            }
        } else {
            frontTarget = frTicks + frontCorrection;
            backTarget = brTicks + backCorrection;
            while (frontRight.getCurrentPosition() < frontTarget && backRight.getCurrentPosition() < backTarget) {
                setDrivePower(dirPower, 0, dirPower, 0);
            }
        }


        System.out.println("Front right ticks: " + frTicks);
        System.out.println("Front left ticks: " + flTicks);
        System.out.println("Back right ticks: " + brTicks);
        System.out.println("Back left ticks: " + blTicks);
        System.out.println("Correction: " + frontCorrection + " " + backCorrection);

        HALT();
    }

    public String cargoDetection(){
        // Cargo detection
        // The less the distance from the ground subtraction the higher object we are possessing
        currentCargoDistance = cargoDetector.getDistance(DistanceUnit.CM);
        if (3.45 < collectorBoxHeight - currentCargoDistance && collectorBoxHeight - currentCargoDistance < 7.5) {
            return "Ball";
        }
        else if(1 < collectorBoxHeight - currentCargoDistance && collectorBoxHeight - currentCargoDistance < 3.45) {
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
        collectorBoxHeight = cargoDetector.getDistance(DistanceUnit.CM);

        // Set the zero power behavior of the motors.
        // We don't want them to slide after every trajectory or else we will lose accuracy.
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Run using encoders!!!
        arm.setRunMode(Motor.RunMode.PositionControl);
        resetEncoders();

        // Initialize the new detector
//        Detector detector = new Detector(hardwareMap);
    }
}