package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private long clawTime = 0;
    private final int ticks_per_revolution = 1120;
    private final double wheel_radius = 75/2;
    private final double wheel_circumference = Math.PI * Math.pow(wheel_radius, 2);
    private final double center_to_wheel = 21;
    private final double turn_circumference = Math.PI * Math.pow(center_to_wheel, 2);

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    DcMotorEx frontLeft;
    DcMotorEx backLeft;
    DcMotorEx frontRight;
    DcMotorEx backRight;
    DcMotorEx collector;
    DcMotorEx armClaw;
    DcMotorEx duckSpinner1;
    DcMotorEx duckSpinner2;
    BNO055IMU imu;
    TouchSensor touchSensorSideLeft;
    TouchSensor touchSensorSideRight;
    TouchSensor touchSensorFrontLeft;
    
    public enum Axis {
        X,
        Y,
        Z
    }

    public enum Direction {
        LEFT,
        RIGHT,
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
        touchSensorSideLeft = hardwareMap.get(TouchSensor.class, "leftSideTouch");
        touchSensorSideRight = hardwareMap.get(TouchSensor.class, "rightSideTouch");
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
    

    private void STOP() {
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }

    public boolean isMoving(){
        if (frontRight.isBusy() || backRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy()){
            return true;
        }
        else {
            return false;
        }
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
        long timeMillis = 0;
        int targetTicks = (ticks_per_revolution / wheel_circumference) * targetDistance;
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
    }

    public void drive(Direction dir, double power, double targetDistance) {
        // TODO: adjust gain (almost done)
        double gain = 0.125;
        double currentDistance = 0;
        float targetAngle = getIMUAngle(Axis.Z);
        double rotationsNeeded = targetDistance / wheel_circumference;
        int targetTicks = (int) (rotationsNeeded * ticks_per_revolution);
        
        runToPos(); //is this the right placement???

        while (isMoving()) {
            linearOpMode.telemetry.addData("Robot is moving", dir, power, targetDistance);
            linearOpMode.telemetry.update();

            switch (dir) {
                case FORWARDS:
                    setTargetPos(targetTicks, targetTicks, targetTicks, targetTicks);
                    if (targetTicks > currentDistance && linearOpMode.opModeIsActive()) {
                        currentDistance = (frontRight.getCurrentPosition() - prevTicks) / ticks_per_rev * wheel_circumference;
                        linearOpMode.telemetry.addData("current distance", currentDistance);
                        linearOpMode.telemetry.update();
                        float currentAngle = getIMUAngle(Axis.Z);
                        double correction = (currentAngle - targetAngle) * gain;
                        setAllPower(power - correction, -power, power - correction, -power);
                    } else {
                        setAllPower(power, power, power, power);
                    }
                case BACKWARDS:
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
        }
        resetEncoders();
        linearOpMode.telemetry.addData("Robot has moved", targetDistance, "successfuly!");
        linearOpMode.telemetry.update();
        
    public void turn(Direction dir, double power, double degrees) {
        int ticksToTurn = degrees / 360 * turn_circumference;
        setAllPower(power, power, power, power); // Doesn't matter on direction run to pos will sort it out!
        switch (dir){
            case LEFT:
                setTargetPos(-ticksToTurn, -ticksToTurn, ticksToTurn, ticksToTurn);
                break;
            case RIGHT:
                setTargetPos(ticksToTurn, ticksToTurn, -ticksToTurn, -ticksToTurn);
                break;
            }
        runtoPos();
        while (isMoving()) {
            linearOpMode.telemetry.addData("Robot is turning", dir, power, degrees);
            linearOpMode.telemetry.update();
        }
        linearOpMode.telemetry.addData("Robot has turned", degrees, "successfuly!");
        linearOpMode.telemetry.update();
        resetEncoders();
    }

    public void moveClaw(Position pos) {
        long timeMillis = 0;
        //TODO: Calibrate the time needed for each of the 3 levels
        long lowTime = 100;
        long midTime = 200;
        long highTime = 300;
        long newTime;
        switch(pos) {
            case LOW:
                newTime = lowTime-clawTime;
                if(newTime > 0) {
                    while(System.currentTimeMillis()+newTime > timeMillis && linearOpMode.opModeIsActive()) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(1);
                    }
                } else {
                    newTime = clawTime-lowTime;
                    while(System.currentTimeMillis()+newTime > timeMillis && linearOpMode.opModeIsActive()) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(-1);
                    }
                }
                clawTime = newTime;
                armClaw.setPower(0);
                break;
            case MID:
                newTime = midTime-clawTime;
                if(newTime > 0) {
                    while(System.currentTimeMillis()+newTime > timeMillis && linearOpMode.opModeIsActive()) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(1);
                    }
                } else {
                    newTime = clawTime-midTime;
                    while(System.currentTimeMillis()+newTime > timeMillis && linearOpMode.opModeIsActive()) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(-1);
                    }
                }
                clawTime = newTime;
                armClaw.setPower(0);
                break;
            case HIGH:
                newTime = highTime-clawTime;
                if(newTime > 0) {
                    while(System.currentTimeMillis()+newTime > timeMillis && linearOpMode.opModeIsActive()) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(1);
                    }
                } else {
                    newTime = clawTime-highTime;
                    while(System.currentTimeMillis()+newTime > timeMillis && linearOpMode.opModeIsActive()) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(-1);
                    }
                }
                clawTime = newTime;
                armClaw.setPower(0);
                break;
            case DOWN:
                while(System.currentTimeMillis()+clawTime > timeMillis && linearOpMode.opModeIsActive()) {
                    timeMillis = System.currentTimeMillis();
                    armClaw.setPower(-1);
                }
                clawTime = 0;
                break;
        }
    }

    public void intake(Direction dir) {
        long timeToMove = 1000;
        long timeMillis = 0;
        switch(dir){
            case IN:
                while(System.currentTimeMillis()+timeToMove > timeMillis && linearOpMode.opModeIsActive()) {
                    timeMillis = System.currentTimeMillis();
                    collector.setPower(0.75);
                }
                collector.setPower(0);
                break;
            case OUT:
                while(System.currentTimeMillis()+timeToMove > timeMillis && linearOpMode.opModeIsActive()) {
                    timeMillis = System.currentTimeMillis();
                    collector.setPower(-0.75);
                    }
                collector.setPower(0);
                break;
            }
    }

    public void duckSpin(long timeToSpin) {
        // TODO: calibrate time to spin.
        long timeMillis = 0;
        while(System.currentTimeMillis()+timeToSpin > timeMillis && linearOpMode.opModeIsActive()) {
            timeMillis = System.currentTimeMillis();
            duckSpinner1.setPower(1);
            duckSpinner2.setPower(1);
        }
        duckSpinner1.setPower(0);
        duckSpinner2.setPower(0);
    }

    // Class constructor.
    // Important init code. Modify only if needed.
    public Robot(List<DcMotorEx> motors, LinearOpMode linearOpMode) {
        hardwareMap = linearOpMode.hardwareMap;
        linearOpMode = linearOpMode;
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
        fronRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

}