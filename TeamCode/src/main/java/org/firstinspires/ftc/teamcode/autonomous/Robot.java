package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
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
import org.firstinspires.ftc.teamcode.autonomous.vision.TseDetector;
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
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor collector;
    DcMotor arm;
    MotorGroup duckSpinners;
    BNO055IMU imu;
    SensorRevTOFDistance cargoDetector;
    TouchSensor touchSensorSideRight;
    TouchSensor touchSensorSideLeft;
    TouchSensor touchSensorFrontLeft;
    
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    //Constants
    public static double driveTicksPerRev = 1120.0;
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

    // Powers
    public double correctedCappedPower = 0;
    public double cappedPower = 0;

    // Ticks completion booleans
    public boolean frCompleted = false;
    public boolean flCompleted = false;
    public boolean brCompleted = false;
    public boolean blCompleted = false;
    public int frCurrentTicks;
    public int flCurrentTicks;
    public int brCurrentTicks;
    public int blCurrentTicks;
    public double frPower;
    public double flPower;
    public double brPower;
    public double blPower;

    // cargoDetector
    public String detectedCargo = "None";
    public double cubeHeight= 5.08;
    public double ballHeight = 6.99;
    public double duckHeight = 5.4;
    public double currentDistance = 0;
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

    public String inContact() {
        touchSensorSideRight = hardwareMap.get(TouchSensor.class, "rightSideTouch");
        touchSensorSideLeft = hardwareMap.get(TouchSensor.class, "leftSideTouch");
        touchSensorFrontLeft = hardwareMap.get(TouchSensor.class, "frontLeftTouch");

        if (touchSensorSideLeft.isPressed()) {
            return "Left side";
        } else if (touchSensorSideRight.isPressed()) {
            return "Right side";
        } else if (touchSensorFrontLeft.isPressed()){
            return "Front side";
        } else {
            return "Not In Contact";
        }
    }

    private void initIMU() {
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled = true;
        params.loggingTag = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
    }

    private void initDistanceSensor(){
        cargoDetector = new SensorRevTOFDistance(hardwareMap, "cargoDetector");
    }

    public float getIMUAngle(Axis axis) {
        Orientation lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        switch(axis) {
            case X:
                return lastAngles.thirdAngle;
            case Y:
                return lastAngles.secondAngle;
            case Z:
                return lastAngles.firstAngle;   
        }
        return 0;
    }

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

    public TseDetector.Location getTsePos() {
        TseDetector detector;
        detector = new TseDetector();
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
        TseDetector.Location location = detector.getLocation();
        return location;
    }
    

    private void HALT() {
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }

    public boolean isMoving(){
        return frontRight.isBusy() || frontLeft.isBusy() || backRight.isBusy() || backLeft.isBusy();
    }

    public void resetEncoders() {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTargetPos(double frpos, double flpos, double brpos, double blpos) {
        frontRight.setTargetPosition((int) frpos);
        frontLeft.setTargetPosition((int) flpos);
        backRight.setTargetPosition((int) brpos);
        backLeft.setTargetPosition((int) blpos);
    }

    public void runToPos() {
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setAllPower(double frPower, double flPower, double brPower, double blPower) {
        frontRight.setPower(frPower);
        frontLeft.setPower(flPower);
        backRight.setPower(brPower);
        backLeft.setPower(blPower);
    }

    public void strafe(Direction dir, double power, double targetDistance) {
        targetRotations = targetDistance / wheelCircumference;
        targetTicks = targetRotations * driveTicksPerRev;
        resetEncoders();
        switch (dir) {
            case LEFT:
                while (targetTicks > currentTicks && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.Z);
                    correction = (targetAngle - currentAngle) * strafeGain;
                    cappedPower = Range.clip(power, -1, 1);
                    correctedCappedPower = Range.clip(power - correction, -1, 1);
                    setAllPower(-correctedCappedPower, -cappedPower, correctedCappedPower, cappedPower);
                    currentTicks = frontRight.getCurrentPosition();
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", -correctedCappedPower + " " + -cappedPower + " " + correctedCappedPower + " " + cappedPower);
                    linearOpMode.telemetry.addData("Current Ticks: ", currentTicks);
                    linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance);
                    linearOpMode.telemetry.update();
                }
            case RIGHT:
                while (targetTicks > currentTicks && linearOpMode.opModeIsActive()) {
                    currentAngle = getIMUAngle(Axis.Z);
                    correction = (targetAngle - currentAngle) * strafeGain;
                    cappedPower = -Range.clip(power, -1, 1);
                    correctedCappedPower = -Range.clip(power - correction, -1, 1);
                    setAllPower(correctedCappedPower, cappedPower, -correctedCappedPower, -cappedPower);
                    currentTicks = frontRight.getCurrentPosition();
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

    public void drive(Direction dir, double power, double targetDistance) {
        // TODO: adjust gain (almost done)
        targetAngle = getIMUAngle(Axis.Z);
        targetRotations = targetDistance / wheelCircumference;
        targetTicks = targetRotations * driveTicksPerRev;
        resetEncoders();
        switch (dir) {
            case FORWARDS:
                while (!frCompleted && !flCompleted && !brCompleted && !blCompleted && linearOpMode.opModeIsActive()) {
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
                    if (targetTicks <= frCurrentTicks) {
                        frCompleted = true;
                        frPower = 0;
                    }
                    if (targetTicks <= frCurrentTicks) {
                        flCompleted = true;
                        flPower = 0;
                    }
                    if (targetTicks <= frCurrentTicks) {
                        brCompleted = true;
                        brPower = 0;
                    }
                    if (targetTicks <= frCurrentTicks) {
                        blCompleted = true;
                        blPower = 0;
                    }
                    setAllPower(frPower, flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", correctedCappedPower + " " + cappedPower + " " + correctedCappedPower + " " + cappedPower);
                    linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance);
                    linearOpMode.telemetry.update();
                }
            case BACKWARDS:
                while (!frCompleted && !flCompleted && !brCompleted && !blCompleted && linearOpMode.opModeIsActive()) {
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
                    if (targetTicks <= frCurrentTicks) {
                        frCompleted = true;
                        frPower = 0;
                    }
                    if (targetTicks <= frCurrentTicks) {
                        flCompleted = true;
                        flPower = 0;
                    }
                    if (targetTicks <= frCurrentTicks) {
                        brCompleted = true;
                        brPower = 0;
                    }
                    if (targetTicks <= frCurrentTicks) {
                        blCompleted = true;
                        blPower = 0;
                    }
                    setAllPower(frPower, flPower, brPower, blPower);
                    linearOpMode.telemetry.addData("Correction: ", correction);
                    linearOpMode.telemetry.addData("Current Angle: ", currentAngle);
                    linearOpMode.telemetry.addData("Current Power: ", correctedCappedPower + " " + cappedPower + " " + correctedCappedPower + " " + cappedPower);
                    linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
                    linearOpMode.telemetry.addData("Robot is moving: ", String.valueOf(dir), " with a power of ", power, " for ", targetDistance);
                    linearOpMode.telemetry.update();
                }
                linearOpMode.telemetry.addData("Robot has moved: ", String.valueOf(targetDistance), " ", dir);
                linearOpMode.telemetry.update();
        }
        while (isMoving()) {}
    }
        
    public void turn(Direction dir, double power, double degrees) {
        targetRotations = degrees / 360 * turnCircumference;
        ticksToTurn = targetRotations * driveTicksPerRev;
        resetEncoders();
        setAllPower(power, power, power, power); // Doesn't matter on direction run to pos will sort it out!
        switch (dir){
            case LEFT:
                setTargetPos(-ticksToTurn, -ticksToTurn, ticksToTurn, ticksToTurn);
                break;
            case RIGHT:
                setTargetPos(ticksToTurn, ticksToTurn, -ticksToTurn, -ticksToTurn);
                break;
            }
        runToPos();
        while (isMoving()) {
            linearOpMode.telemetry.addData("Current Power: ", String.valueOf(power));
            linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
            linearOpMode.telemetry.addData("Robot is turning ", String.valueOf(degrees), "to the ", dir, "with a power of ", power);
            linearOpMode.telemetry.addData("Target Ticks to turn: ", ticksToTurn);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("Robot has turned", String.valueOf(degrees), "to the", dir);
        linearOpMode.telemetry.update();
    }

    public void moveArm(Position pos, double power) {
        //TODO: Calibrate the ticks needed for each of the 3 levels
        double lowPosition = 100;
        double midPosition = 200;
        double highPosition = 300;
        arm.setPower(power);
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
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (collector.isBusy()) {
            linearOpMode.telemetry.addData("The arm is moving ", "to the ", pos, " from ", lastClawPosition, " with a power of ", power);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("The arm has moved ", "to the ", pos, " from ", lastClawPosition);
        linearOpMode.telemetry.update();
    }

    public void intake(Direction dir, double power, long timeToMove) {
        long timeMillis = 0;
        switch(dir){
            case IN:
                while(System.currentTimeMillis()+timeToMove > timeMillis && linearOpMode.opModeIsActive()) {
                    timeMillis = System.currentTimeMillis();
                    collector.setPower(power);
                }
                break;
            case OUT:
                while(System.currentTimeMillis()+timeToMove > timeMillis && linearOpMode.opModeIsActive()) {
                    timeMillis = System.currentTimeMillis();
                    collector.setPower(-power);
                }
                break;
            }
            collector.setPower(0);
        }

    public void duckSpin(double power, long timeToSpin) {
        // TODO: calibrate time to spin.
        long timeMillis = 0;
        while(System.currentTimeMillis()+timeToSpin > timeMillis && linearOpMode.opModeIsActive()) {
            timeMillis = System.currentTimeMillis();
            duckSpinners.set(power);
        }
        duckSpinners.set(0);
    }

    public String cargoDetection(){
        // Cargo detection
        // The less the distance from the ground subtraction the higher object we are possessing
        double currentDistance = cargoDetector.getDistance(DistanceUnit.CM);
        if (collectorBoxHeight - cubeHeight < collectorBoxHeight - currentDistance) {
            detectedCargo = "Ball";
        }
        else if(collectorBoxHeight - ballHeight < collectorBoxHeight - currentDistance) {
            detectedCargo = "Cube OR Duck";
        }
        else {
            detectedCargo = "None";
        }
        return detectedCargo;
    }

    // Class constructor.
    // Important initialization code. Modify only if needed.
    public Robot(List<DcMotor> motors, LinearOpMode linearOpMode) {
        hardwareMap = linearOpMode.hardwareMap;
        this.linearOpMode = linearOpMode;
        backLeft = motors.get(0);
        frontLeft = motors.get(1);
        backRight = motors.get(2);
        frontRight = motors.get(3);
        arm = motors.get(4);
        collector = motors.get(5);
        duckSpinners = motors.get(6);
        initDistanceSensor();
        initIMU();

        // Run using encoders!!!
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoders();

        // Fix all the directions of the motors.
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set the zero power behavior of the motors.
        // We don't want them to slide after every trajectory or else we will lose accuracy.
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU remapping axis
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        // Distance sensor and cargo definition
    }
}