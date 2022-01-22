package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private final int ticks_per_rev = 1120;
    private final double radius_of_wheels = 75/2;
    private final double wheels_circumference = Math.PI * Math.pow(radius_of_wheels, 2);
    private final double center_to_wheel = 21;
    private final double turn_circumference = Math.PI * Math.pow(center_to_wheel, 2);

    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor collector;
    DcMotor armClaw;
    DcMotor duckSpinner1;
    DcMotor duckSpinner2;
    BNO055IMU imu;
    TouchSensor touchSensorSideLeft;
    TouchSensor touchSensorSideRight;
    TouchSensor touchSensorFrontRight;
    
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

    // TODO: Place the three touch sensors accordingly
    public String inContact() {
        touchSensorSideLeft = hardwareMap.get(TouchSensor.class, "leftSideTouch");
        touchSensorSideRight = hardwareMap.get(TouchSensor.class, "rightSideTouch");
        touchSensorFrontRight = hardwareMap.get(TouchSensor.class, "frontRightTouch");

        if (touchSensorSideLeft.isPressed()) {
            return "Left";
        } else if (touchSensorSideRight.isPressed()) {
            return "Right";
        } else if (touchSensorFrontRight.isPressed()){
            return "Center";
        } else {
            return null;
        }
    }

    private void initIMU() {
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
    }

    public float getIMUAngle(Axis axis) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        switch(axis) {
            case X:
                return angles.thirdAngle;
            case Y:
                return angles.secondAngle;
            case Z:
                return angles.firstAngle;
                
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
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public boolean isMoving(){
        if (frontLeft.isBusy() || backLeft.isBusy() || frontLeft.isBusy() || backRight.isBusy()){
            return true;
        }
        else{
            return false;
        }
    }

    public void strafe(Direction dir, double power, long time) {
        long timeMillis = 0;
        switch(dir) {
            case LEFT:
                while(System.currentTimeMillis()+time > timeMillis) {
                    timeMillis = System.currentTimeMillis();
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                }
                this.STOP();
                break;
            case RIGHT:
                while(System.currentTimeMillis()+time > timeMillis) {
                    timeMillis = System.currentTimeMillis();
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                }
                this.STOP();
                break;
        }
    }

    public void drive(Direction dir, double power, double targetDistance) {
            // TODO: adjust gain (almost done)
            double gain = 0.125;
            double currentDistance = 0;
            double prevTicks = frontRight.getCurrentPosition();
            float targetAngle = getIMUAngle(Axis.Z);
            switch (dir) {
                case BACKWARDS:
                    while (targetDistance > currentDistance && linearOpMode.opModeIsActive()) {
                        currentDistance = (frontRight.getCurrentPosition() - prevTicks) / ticks_per_rev * wheels_circumference;
                        linearOpMode.telemetry.addData("kkk",frontRight.getCurrentPosition());
                        linearOpMode.telemetry.addData("current distance", currentDistance);
                        linearOpMode.telemetry.update();
                        float currentAngle = getIMUAngle(Axis.Z);
                        double correction = (currentAngle - targetAngle) * gain;
                        frontLeft.setPower(-power);
                        frontRight.setPower(power - correction);
                        backLeft.setPower(-power);
                        backRight.setPower(power - correction);
                    }
                case FORWARDS:
                    while (targetDistance > currentDistance && linearOpMode.opModeIsActive()) {
                        currentDistance = (frontRight.getCurrentPosition() - prevTicks) / ticks_per_rev * wheels_circumference;
                        linearOpMode.telemetry.addData("kkk",frontRight.getCurrentPosition());
                        linearOpMode.telemetry.addData("current distance", currentDistance);
                        linearOpMode.telemetry.update();
                        float currentAngle = getIMUAngle(Axis.Z);
                        double correction = (currentAngle - targetAngle) * gain;
                        frontLeft.setPower(power);
                        frontRight.setPower(-(power - correction));
                        backLeft.setPower(power);
                        backRight.setPower(-(power - correction));
                    }
            }
            STOP();

    }
        
    public void turn(Direction dir, double power, double degrees) {
        double ticksToTurn = degrees / 360 * turn_circumference;
        switch (dir){
            case LEFT:
                while(ticksToTurn > frontRight.getCurrentPosition()) {
                    frontLeft.setPower(power);
                    frontRight.setPower(power);
                    backLeft.setPower(power);
                    backRight.setPower(power);
                }
                this.STOP();
                break;
            case RIGHT:
                while(ticksToTurn > frontRight.getCurrentPosition()) {
                    frontLeft.setPower(-power);
                    frontRight.setPower(-power);
                    backLeft.setPower(-power);
                    backRight.setPower(-power);
                }
                this.STOP();
                break;
            } 
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
                    while(System.currentTimeMillis()+newTime > timeMillis) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(1);
                    }
                } else {
                    newTime = clawTime-lowTime;
                    while(System.currentTimeMillis()+newTime > timeMillis) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(-1);
                    }
                }
                this.clawTime = newTime;
                armClaw.setPower(0);
                break;
            case MID:
                newTime = midTime-clawTime;
                if(newTime > 0) {
                    while(System.currentTimeMillis()+newTime > timeMillis) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(1);
                    }
                } else {
                    newTime = clawTime-midTime;
                    while(System.currentTimeMillis()+newTime > timeMillis) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(-1);
                    }
                }
                this.clawTime = newTime;
                armClaw.setPower(0);
                break;
            case HIGH:
                newTime = highTime-clawTime;
                if(newTime > 0) {
                    while(System.currentTimeMillis()+newTime > timeMillis) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(1);
                    }
                } else {
                    newTime = clawTime-highTime;
                    while(System.currentTimeMillis()+newTime > timeMillis) {
                        timeMillis = System.currentTimeMillis();
                        armClaw.setPower(-1);
                    }
                }
                this.clawTime = newTime;
                armClaw.setPower(0);
                break;
            case DOWN:
                while(System.currentTimeMillis()+clawTime > timeMillis) {
                    timeMillis = System.currentTimeMillis();
                    armClaw.setPower(-1);
                }
                this.clawTime = 0;
                break;
        }
    }

    public void intake(Direction dir) {
        long timeToMove = 1000;
        long timeMillis = 0;
        switch(dir){
            case IN:
                while(System.currentTimeMillis()+timeToMove > timeMillis) {
                    timeMillis = System.currentTimeMillis();
                    collector.setPower(0.6);
                }
                collector.setPower(0);
                break;
            case OUT:
                while(System.currentTimeMillis()+timeToMove > timeMillis) {
                    timeMillis = System.currentTimeMillis();
                    collector.setPower(-0.6);
                    }
                collector.setPower(0);
                break;
            }
    }

    public void duckSpin() {
        // TODO: calibrate time to spin.
        long timeToSpin = 1000;
        long timeMillis = 0;
        while(System.currentTimeMillis()+timeToSpin > timeMillis) {
            timeMillis = System.currentTimeMillis();
            duckSpinner1.setPower(1);
            duckSpinner2.setPower(1);
        }
        duckSpinner1.setPower(0);
        duckSpinner2.setPower(0);
    }

    // Class constructor. -
    // Important init code. Modify only if needed.
    public Robot(List<DcMotor> motors, LinearOpMode linearOpMode) {
        this.hardwareMap = linearOpMode.hardwareMap;
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

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}