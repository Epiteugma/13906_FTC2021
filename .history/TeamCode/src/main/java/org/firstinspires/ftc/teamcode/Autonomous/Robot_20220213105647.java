package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.vision.DuckDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// Item detector based on the ground tape
import org.firstinspires.ftc.teamcode.Autonomous.visionv2.Detector;


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
    BNO055IMU imu;
    SensorRevTOFDistance cargoDetector;
    TouchSensor touchSensorSideRight;
    TouchSensor touchSensorSideLeft;
    TouchSensor touchSensorFrontLeft;
    
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    //Constants
    // TODO: adjust gain (almost done)
    public static double gearRatio = 26.0/10.0;
    public static double driveTicksPerRev = 1120.0 * gearRatio;
    public static double armTickPerRev = 1120.0;
    // Old Wheels
    // public static double wheelRadius = 7/2;
    // New wheels
    public static double wheelRadius = 15/2;
    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
    public static double centerToWheel = 21;
    public static double turnCircumference = 2 * Math.PI * centerToWheel;
    public static double driveGain = 0.05;
    public static double strafeGain = 0.05;
    public static double turnGain = 0.125;

    //Ticks Angles and postiions
    public double lastClawPosition = 0;
    public double targetRotations = 0;
    public double targetTicks = 0;
    public double ticksToTurn = 0;
    public double correction = 0;
    public float targetAngle = 0;
    public float currentAngle = 0;
    public int currentTicks = 0;

    // Powers and error tolerance
    public double driveErrorTolerance = 2; // in ticks
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
    public double collectorBoxHeight = 0;


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
        UP,
        DOWN,
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

    private void initIMU() {
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();
    }

    private void initCargoDetector(){
        cargoDetector = new SensorRevTOFDistance(hardwareMap, "cargoDetector");
        collectorBoxHeight = cargoDetector.getDistance(DistanceUnit.CM);
    }

    public float getIMUAngle(Axis axis) {
        switch(axis) {
            case X:
                return imu.getAngularOrientation().firstAngle;
            case Y:
                return imu.getAngularOrientation().secondAngle;
            case Z:
                return imu.getAngularOrientation().thirdAngle;
        }
        return 0;
    }

    // OLD!!!
    public DuckDetector.Location getDuckPos() {
        DuckDetector detector;
        detector = new DuckDetector();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        DuckDetector.Location location = detector.getLocation();
        return location;
    }

    public Detector.ElementPosition getTsePos() {
        Detector.ElementPosition pos = Detector.ElementPosition.CENTER;
        linearOpMode.telemetry.addData("The shipping element is located at the ", pos);
        linearOpMode.telemetry.update();
        return pos;
    }
    

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
        frontRight.resetEncoder();
        frontLeft.resetEncoder();
        backRight.resetEncoder();
        backLeft.resetEncoder();
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

    public void runToPos() {
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
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
        setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
        switch (dir) {
            case LEFT:
                while (!frontRight.atTargetPosition() || !frontLeft.atTargetPosition() || !backRight.atTargetPosition() || !backLeft.atTargetPosition() && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.Z);
                    correction = (targetAngle - currentAngle) * strafeGain;
                    cappedPower = Range.clip(power, -1, 1);
                    correctedCappedPower = Range.clip(power - correction, -1, 1);
                    frPower = -correctedCappedPower;
                    flPower = -cappedPower;
                    brPower = correctedCappedPower;
                    blPower = cappedPower;
                    frCurrentTicks = frontRight.getCurrentPosition();
                    flCurrentTicks = frontLeft.getCurrentPosition();
                    brCurrentTicks = backRight.getCurrentPosition();
                    blCurrentTicks = backLeft.getCurrentPosition();
                    if (frontRight.atTargetPosition()) {
                        frPower = 0;
                    }
                    if (frontLeft.atTargetPosition()) {
                        flPower = 0;
                    }
                    if (backRight.atTargetPosition()) {
                        brPower = 0;
                    }
                    if (backLeft.atTargetPosition()) {
                        blPower = 0;
                    }
                    setDrivePower(frPower, flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", "FR: " + frPower + " FL: " + flPower + " BR: " + brPower + " BL: " + blPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", "FR: "+ frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
                    linearOpMode.telemetry.update();
                }
            case RIGHT:
                while (!frontRight.atTargetPosition() || !frontLeft.atTargetPosition() || !backRight.atTargetPosition() || !backLeft.atTargetPosition() && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.Z);
                    correction = (targetAngle - currentAngle) * strafeGain;
                    cappedPower = -Range.clip(power, -1, 1);
                    correctedCappedPower = -Range.clip(power - correction, -1, 1);
                    frPower = correctedCappedPower;
                    flPower = cappedPower;
                    brPower = -correctedCappedPower;
                    blPower = -cappedPower;
                    frCurrentTicks = frontRight.getCurrentPosition();
                    flCurrentTicks = frontLeft.getCurrentPosition();
                    brCurrentTicks = backRight.getCurrentPosition();
                    blCurrentTicks = backLeft.getCurrentPosition();
                    if (frontRight.atTargetPosition()) {
                        frPower = 0;
                    }
                    if (frontLeft.atTargetPosition()) {
                        flPower = 0;
                    }
                    if (backRight.atTargetPosition()) {
                        brPower = 0;
                    }
                    if (backLeft.atTargetPosition()) {
                        blPower = 0;
                    }
                    setDrivePower(frPower, flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", correctedCappedPower + " " + cappedPower + " " + -correctedCappedPower + " " + -cappedPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", currentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
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
        targetAngle = getIMUAngle(Axis.Z);
        targetRotations = targetDistance / wheelCircumference;
        targetTicks = targetRotations * driveTicksPerRev;
        resetEncoders();
        setDriveTargetPos(targetTicks, targetTicks, targetTicks, targetTicks);
        setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
        switch (dir) {
            case FORWARDS:
                while (!frontRight.atTargetPosition() || !frontLeft.atTargetPosition() || !backRight.atTargetPosition() || !backLeft.atTargetPosition() && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.Z);
                    if (Math.abs(targetAngle - currentAngle) > 3) {
                        correction = (targetAngle - currentAngle) * driveGain;
                    }
                    else {
                        correction = 1;
                    }
                    cappedPower = Range.clip(power + correction, -1, 1);
                    correctedCappedPower = Range.clip(power - correction, -1, 1);
                    frPower = correctedCappedPower;
                    flPower = cappedPower;
                    brPower = correctedCappedPower;
                    blPower = cappedPower;
                    frCurrentTicks = frontRight.getCurrentPosition();
                    flCurrentTicks = frontLeft.getCurrentPosition();
                    brCurrentTicks = backRight.getCurrentPosition();
                    blCurrentTicks = backLeft.getCurrentPosition();
                    if (frontRight.atTargetPosition()) {
                        frPower = 0;
                    }
                    if (frontLeft.atTargetPosition()) {
                        flPower = 0;
                    }
                    if (backRight.atTargetPosition()) {
                        brPower = 0;
                    }
                    if (backLeft.atTargetPosition()) {
                        blPower = 0;
                    }
                    setDrivePower(frPower, flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", "FR: " + correctedCappedPower + " FL: " + cappedPower + " BR: " + correctedCappedPower + " BL: " + cappedPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", "FR: " + frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
                    linearOpMode.telemetry.update();
                }
            case BACKWARDS:
                while (!frontRight.atTargetPosition() || !frontLeft.atTargetPosition() || !backRight.atTargetPosition() || !backLeft.atTargetPosition() && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.Z);
                    correction = (targetAngle - currentAngle) * driveGain;
                    cappedPower = -Range.clip(power, -1, 1);
                    correctedCappedPower = -Range.clip(power - correction, -1, 1);
                    frPower = correctedCappedPower;
                    flPower = cappedPower;
                    brPower = correctedCappedPower;
                    blPower = cappedPower;
                    frCurrentTicks = frontRight.getCurrentPosition();
                    flCurrentTicks = frontLeft.getCurrentPosition();
                    brCurrentTicks = backRight.getCurrentPosition();
                    blCurrentTicks = backLeft.getCurrentPosition();
                    if (frontRight.atTargetPosition()) {
                        frPower = 0;
                    }
                    if (frontLeft.atTargetPosition()) {
                        flPower = 0;
                    }
                    if (backRight.atTargetPosition()) {
                        brPower = 0;
                    }
                    if (backLeft.atTargetPosition()) {
                        blPower = 0;
                    }
                    setDrivePower(frPower, flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", "FR: " + correctedCappedPower + " FL: " + cappedPower + " BR: " + correctedCappedPower + " BL: " + cappedPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", "FR: " + frCurrentTicks + " FL: " + flCurrentTicks + " BR: " + brCurrentTicks + " BL: " + blCurrentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance + "cm");
                    linearOpMode.telemetry.update();
                }
        while (isMoving()) {
            linearOpMode.telemetry.addData("Robot is moving" ," (when it shouldn't)");
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("Robot has moved: ", String.valueOf(targetDistance) + "cm " + dir);
        linearOpMode.telemetry.update();
        }
    }
        
    public void turn(Direction dir, double power, double degrees) {
        frPower = flPower = brPower = blPower = power;
        targetRotations = degrees / 360 * turnCircumference;
        ticksToTurn = targetRotations * driveTicksPerRev;
        resetEncoders();
        setDriveTolerance(driveErrorTolerance, driveErrorTolerance, driveErrorTolerance, driveErrorTolerance);
        switch (dir){
            case LEFT:
                setDriveTargetPos(-ticksToTurn, -ticksToTurn, ticksToTurn, ticksToTurn);
                break;
            case RIGHT:
                setDriveTargetPos(ticksToTurn, ticksToTurn, -ticksToTurn, -ticksToTurn);
                break;
            }
        while (!frontRight.atTargetPosition() || !frontLeft.atTargetPosition() || !backRight.atTargetPosition() || !backLeft.atTargetPosition() && linearOpMode.opModeIsActive()) {
            if (frontRight.atTargetPosition()) {
                frPower = 0;
            }
            if (frontLeft.atTargetPosition()) {
                flPower = 0;
            }
            if (backRight.atTargetPosition()) {
                brPower = 0;
            }
            if (backLeft.atTargetPosition()) {
                blPower = 0;
            }
        setDrivePower(frPower, flPower, brPower, blPower);
        }
        while (isMoving()) {
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
        double lowPosition = 100;
        double midPosition = 200;
        double highPosition = 300;
        arm.setPositionTolerance(2);
        switch(pos) {
            case LOW:
                if (lowPosition > lastClawPosition) {
                    lowPosition = lowPosition + lastClawPosition;
                }
                else {
                    lowPosition = lowPosition - lastClawPosition;
                }
                arm.setTargetPosition((int) (lowPosition * armTickPerRev));
                lastClawPosition = lowPosition;
                break;
            case MID:
                if (midPosition > lastClawPosition) {
                    midPosition = midPosition + lastClawPosition;
                }
                else {
                    midPosition = midPosition - lastClawPosition;
                }
                arm.setTargetPosition((int) (midPosition * armTickPerRev));
                lastClawPosition = midPosition;
                break;
            case HIGH:
                // No need to check if above it will never be above the highest possible position
                highPosition = highPosition - lastClawPosition;
                arm.setTargetPosition((int) (highPosition * armTickPerRev));
                lastClawPosition = highPosition;
                break;
            case DOWN:
                arm.setTargetPosition((int) (-lastClawPosition * armTickPerRev));
                break;
        }
        while (!collector.atTargetPosition()) {
            arm.set(power);
            linearOpMode.telemetry.addData("The arm is moving ", "to the ", pos, " from ", lastClawPosition, " with a power of ", power);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("The arm has moved ", "to the ", pos, " from ", lastClawPosition);
        linearOpMode.telemetry.update();
    }

    public void intake(Direction dir, double power) {
        switch(dir){
            case IN:
                while(cargoDetection() == "None"){
                    linearOpMode.telemetry.addData("We are still NOT in possession of cargo...", "None");
                    linearOpMode.telemetry.update();
                    collector.set(power);
                }
                break;
            case OUT:
                while(cargoDetection() != "None"){
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
            timeMillis = System.currentTimeMillis();
            duckSpinners.set(power);
        }
        duckSpinners.stopMotor();
    }

    public String cargoDetection(){
        // Cargo detection
        // The less the distance from the ground subtraction the higher object we are possessing
        double currentCargoDistance = cargoDetector.getDistance(DistanceUnit.CM);
        if (collectorBoxHeight - cubeHeight < collectorBoxHeight - currentCargoDistance) {
            detectedCargo = "Ball";
        }
        else if(collectorBoxHeight - ballHeight < collectorBoxHeight - currentCargoDistance) {
            detectedCargo = "Cube OR Duck";
        }
        else {
            detectedCargo = "None";
        }
        return detectedCargo;
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
        initCargoDetector();
        initIMU();

        // Fix all the directions of the motors.
        
        backRight.setInverted(true);

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
        Detector detector = new Detector(hardwareMap);
    }
}