package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.DuckDetector;
import org.firstinspires.ftc.teamcode.vision.TseDetector;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Robot {
    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;
    DcMotorEx collector;
    DcMotorEx armClaw;
    DcMotorEx duckSpinner1;
    DcMotorEx duckSpinner2;
    BNO055IMU imu;
    TouchSensor touchSensorSideRight;
    TouchSensor touchSensorSideLeft;
    TouchSensor touchSensorFrontLeft;
    
    // Tunable variables to tune from the dashboard, must be public and static so the dashboard can access them.
    // Always have to be in cm!!!
    public int ticksPerRevolution = 1120;
    public static double wheelRadius = 7.5/2;
    public static double wheelCircumference = 2 * Math.PI * wheelRadius;
    public static double centerToWheel = 21;
    public static double turnCircumference = 2 * Math.PI * centerToWheel;

    public static double gain = 0.125;
    public static int lastClawPosition = 0;
    public static int targetTicks =0;
    public static int ticksToTurn = 0;

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
        webcam.openCameraDevice();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        DuckDetector.Location location = detector.getLocation();
        return location;
    }

    public TseDetector.Location getTsePos() {
        TseDetector detector;
        detector = new TseDetector();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTargetPos(int frpos, int flpos, int brpos, int blpos) {
        frontRight.setTargetPosition(frpos);
        frontLeft.setTargetPosition(flpos);
        backRight.setTargetPosition(brpos);
        backLeft.setTargetPosition(blpos);
    }

    public void runToPos() {
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setAllPower(double frpower, double flpower, double brpower, double blpower) {
        frontRight.setPower(frpower);
        frontLeft.setPower(flpower);
        backRight.setPower(brpower);
        backLeft.setPower(blpower);
    }

    public void strafe(Direction dir, double power, double targetDistance) {
        targetTicks = (int) ((ticksPerRevolution / wheelCircumference) * targetDistance);

        setAllPower(power, power, power, power); // Doesn't matter on direction run to pos will sort it out!
        switch(dir) {
            case LEFT:
                setTargetPos(targetTicks, -targetTicks, -targetTicks, targetTicks);
                break;
            case RIGHT:
                setTargetPos(-targetTicks, targetTicks, targetTicks, -targetTicks);
                break;
        }
        runToPos();
        while (isMoving()) {
            linearOpMode.telemetry.addData("Robot is strafing ", String.valueOf(dir), " ", power, " ", targetDistance);
            linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("Robot has strafed ", String.valueOf(targetDistance), " to the ", dir);
        linearOpMode.telemetry.update();
    }

    public void drive(Direction dir, double power, double targetDistance){
            // TODO: adjust gain (almost done)
            double currentDistance = 0;
            float targetAngle = getIMUAngle(Axis.Z);
            double rotationsNeeded = targetDistance / wheelCircumference;
            targetTicks = (int) (rotationsNeeded * ticksPerRevolution);
            linearOpMode.telemetry.addData("Target Ticks: ", targetTicks);
            linearOpMode.telemetry.update();

            setAllPower(power,power,power,power); // Temp power

            boolean firstRun = true;

            while (isMoving() || firstRun) {
                firstRun = false;
                linearOpMode.telemetry.addData("Robot is moving to the", String.valueOf(dir), " with a power of ", power, " for ", targetDistance);
                linearOpMode.telemetry.update();

                switch (dir) {
                    case FORWARDS:
                        setTargetPos(targetTicks, targetTicks, targetTicks, targetTicks);
                        if (targetTicks > currentDistance && linearOpMode.opModeIsActive()) {
                            currentDistance = (frontRight.getCurrentPosition()) / ticksPerRevolution * wheelCircumference;
                            linearOpMode.telemetry.addData("current distance", currentDistance);
                            linearOpMode.telemetry.update();
                            float currentAngle = getIMUAngle(Axis.Z);
                            double correction = (currentAngle - targetAngle) * gain;
                            setAllPower(power - correction, -power, power - correction, -power);
                        } else {
                            setAllPower(power, power, power, power);
                        }
                    case BACKWARDS:
                        setTargetPos(-targetTicks, -targetTicks, -targetTicks, -targetTicks);
                        if (targetTicks > currentDistance && linearOpMode.opModeIsActive()) {
                            currentDistance = frontRight.getCurrentPosition();
                            linearOpMode.telemetry.addData("current distance", currentDistance);
                            linearOpMode.telemetry.update();
                            float currentAngle = getIMUAngle(Axis.Z);
                            double correction = (currentAngle - targetAngle) * gain;
                            setAllPower(-(power - correction), power, -(power - correction), power);
                        } else {
                            setAllPower(-power, -power, -power, -power);
                        }
                }
                runToPos(); //is this the right placement???
            }
            resetEncoders();
            linearOpMode.telemetry.addData("Robot has moved", String.valueOf(targetDistance), " ", dir);
            linearOpMode.telemetry.update();
        }
        
    public void turn(Direction dir, double power, double degrees) {
        ticksToTurn = (int) (degrees / 360 * turnCircumference);
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
            linearOpMode.telemetry.addData("Robot is turning ", String.valueOf(degrees), "to the ", dir, "with a power of ", power);
            linearOpMode.telemetry.addData("Target Ticks to turn: ", ticksToTurn);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("Robot has turned", String.valueOf(degrees), "to the", dir);
        linearOpMode.telemetry.update();
        resetEncoders();
    }

    public void moveClaw(Position pos, double power) {
        //TODO: Calibrate the ticks needed for each of the 3 levels
        int lowPosition = 100;
        int midPosition = 200;
        int highPosition = 300;
        collector.setPower(power);
        switch(pos) {
            case LOW:
                if (lowPosition > lastClawPosition) {
                    lowPosition = lowPosition + lastClawPosition;
                }
                else {
                    lowPosition = lowPosition - lastClawPosition;
                }
                collector.setTargetPosition(lowPosition * ticksPerRevolution);
                lastClawPosition = lowPosition;
                break;
            case MID:
                if (midPosition > lastClawPosition) {
                    midPosition = midPosition + lastClawPosition;
                }
                else {
                    midPosition = midPosition - lastClawPosition;
                }
                collector.setTargetPosition(midPosition);
                lastClawPosition = midPosition;
                break;
            case HIGH:
                // No need to check if above it will never be above the highest possible position
                highPosition = highPosition - lastClawPosition;
                collector.setTargetPosition(highPosition * ticksPerRevolution);
                lastClawPosition = highPosition;
                break;
            case DOWN:
                collector.setTargetPosition(-lastClawPosition * ticksPerRevolution);
                break;
        }
        collector.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        while (collector.isBusy()) {
            linearOpMode.telemetry.addData("The arm is moving ", "to the ", pos, " from ", lastClawPosition, " with a power of ", power);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("The arm has moved ", "to the ", pos, " from ", lastClawPosition);
        linearOpMode.telemetry.update();
    }

    public void intake(Direction dir, double power) {
        long timeToMove = 1000;
        long timeMillis = 0;
        switch(dir){
            case IN:
                while(System.currentTimeMillis()+timeToMove > timeMillis && linearOpMode.opModeIsActive()) {
                    timeMillis = System.currentTimeMillis();
                    collector.setPower(power);
                }
                collector.setPower(0);
                break;
            case OUT:
                while(System.currentTimeMillis()+timeToMove > timeMillis && linearOpMode.opModeIsActive()) {
                    timeMillis = System.currentTimeMillis();
                    collector.setPower(-power);
                    }
                collector.setPower(0);
                break;
            }
    }

    public void duckSpin(long timeToSpin, double power) {
        // TODO: calibrate time to spin.
        long timeMillis = 0;
        while(System.currentTimeMillis()+timeToSpin > timeMillis && linearOpMode.opModeIsActive()) {
            timeMillis = System.currentTimeMillis();
            duckSpinner1.setPower(power);
            duckSpinner2.setPower(power);
        }
        duckSpinner1.setPower(0);
        duckSpinner2.setPower(0);
    }

    // Class constructor.
    // Important initialization code. Modify only if needed.
    public Robot(List<DcMotorEx> motors, LinearOpMode linearOpMode) {
        hardwareMap = linearOpMode.hardwareMap;
        this.linearOpMode = linearOpMode;
        backLeft = motors.get(0);
        frontLeft = motors.get(1);
        backRight = motors.get(2);
        frontRight = motors.get(3);
        armClaw = motors.get(4);
        collector = motors.get(5);
        duckSpinner1 = motors.get(6);
        duckSpinner2 = motors.get(7);
        initIMU();

        // Fix all the directions of the motors.
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Run using encoders!!!
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set the zero power behavior of the motors.
        // We don't want them to slide after every trajectory or else we will lose accuracy.
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}