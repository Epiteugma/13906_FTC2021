package org.firstinspires.ftc.teamcode.autonomous.opmodes.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autonomous.vision.TseDetector;
import org.firstinspires.ftc.teamcode.Configurable;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Red Right", group="FTC22Auto")
public class Right extends LinearOpMode {
    TseDetector detector;
    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    Motor arm;
    Motor collector;
    Motor duckSpinner;
    DistanceSensor cargoDetector;
    DistanceSensor backDistance;
    TouchSensor armTouchSensor;
    BNO055IMU imu;

    MecanumDriveTrain driveTrain;

    int timer = 30;

    public void initHardware() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");
        arm = new Motor(hardwareMap, "arm");
        collector = new Motor(hardwareMap, "collector");
        duckSpinner = new Motor(hardwareMap, "duckSpinner");

        // Motor reversing
        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setTargetPosition(0);
        arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setHoldPosition(true);

        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");
        cargoDetector = hardwareMap.get(DistanceSensor.class, "cargoDetector");
        armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouch");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    private void lowerArm() {
        arm.setHoldPosition(false);
        arm.setTargetPosition(0);
        arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (!armTouchSensor.isPressed()) {
            arm.setPower(1);
        }
        arm.resetEncoder();
        arm.setPower(0);
    }

    private void lowerArmAsync() {
        arm.setHoldPosition(false);
        new Thread(() -> {
            arm.setTargetPosition(0);
            arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (!armTouchSensor.isPressed()) {
                arm.setPower(1);
            }
            arm.resetEncoder();
            arm.setPower(0);
        }).start();
    }

    private void collectCube(double power) {
        double initialDistance = cargoDetector.getDistance(DistanceUnit.CM);
        double averageTicks = (Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition())) / 4.0;
        frontLeft.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        collector.setPower(-1);
        collector.resetStallDetection();
        double currentAngle = imu.getAngularOrientation().firstAngle;
        while (initialDistance - cargoDetector.getDistance(DistanceUnit.CM) < 2 && !collector.isStalled()) {
            Logger.addData("Initial Distance: " + initialDistance);
            Logger.addData("Distance: " + cargoDetector.getDistance(DistanceUnit.CM));
            Logger.update();
            driveTrain.driveCM(25, 0.2);
            driveTrain.turn(currentAngle - 8, 0.1, 1);
            driveTrain.turn(currentAngle + 8, 0.1, 1);
        }
        collector.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Go back to initial position.
        driveTrain.drive((int) (-averageTicks * 0.2), 0.2);
    }

    private void releaseCube(double collectorPower) {
        while(!collector.runToPosition(Configurable.disposeTicks,collectorPower)){
            collectorPower += 0.05;
            Logger.addData("Collector Power: " + collectorPower);
            Logger.update();
        }
    }

    private void driveToShippingHub(double power) {
        driveTrain.runOnEncoders();
        while(backDistance.getDistance(DistanceUnit.CM) < Configurable.distanceToShippingHub) {
            frontLeft.setPower(-power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(-power);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        driveTrain.turn(0, 0.1, 1);
    }

    private void driveBackWallDistance(double distance) {
        driveTrain.runOnEncoders();
        while (backDistance.getDistance(DistanceUnit.CM) > distance) {
            Logger.addData("Distance: " + backDistance.getDistance(DistanceUnit.CM));
            frontLeft.setPower(0.2);
            frontRight.setPower(0.2);
            backLeft.setPower(0.2);
            backRight.setPower(0.2);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void startTimer() {
        new Thread(() -> {
            double prevTime = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() > prevTime+1000) {
                timer--;
                prevTime = System.currentTimeMillis();
            }
        });
    }

    @Override
    public void runOpMode() {
        initHardware();

        Logger.setTelemetry(telemetry);
        driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight, imu);
        driveTrain.ratio = Configurable.driveGearRatio;
        driveTrain.wheelRadius = Configurable.wheelRadius;

        detector = new TseDetector();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

        waitForStart();
        startTimer();
        TseDetector.Location itemPos = detector.getLocation(this);
        Logger.addData("Detected Cargo: " + itemPos);
        Logger.update();

        driveTrain.driveCM(15, 0.4);
        driveTrain.turn(90, 0.1, 1);
        driveTrain.driveCM(72, 0.2);
        driveTrain.turn(0, 0.1, 1);
        switch (itemPos) {
            case LEFT:
                arm.runToPositionAsync(Configurable.armLowPosition, 1);
                break;
            case RIGHT:
                arm.runToPositionAsync(Configurable.armHighPosition, 1);
                break;
            case CENTER:
                arm.runToPositionAsync(Configurable.armMidPosition, 1);
                break;
        }
        driveToShippingHub(0.2);
        releaseCube(Configurable.disposeLowSpeed);
        driveBackWallDistance(50);
        arm.runToPositionAsync(Configurable.armHighPosition, 1);
        driveTrain.turn(-90, 0.1, 1);
        driveTrain.driveCM(250, 0.6);
        lowerArm();
        driveTrain.turn(-105, 0.1, 1);
        collectCube(0.2);
        if(timer < 5) {
            this.stop();
        }
        arm.runToPositionAsync(Configurable.armHighPosition, 1);
        driveTrain.turn(90, 0.1, 1);
        driveTrain.driveCM(300, 0.6);
        driveTrain.turn(0, 0.1, 1);
        driveToShippingHub(0.2);
        releaseCube(Configurable.disposeHighSpeed);
        while(backDistance.getDistance(DistanceUnit.CM) > 20) {
            frontLeft.setPower(0.2);
            frontRight.setPower(0.2);
            backLeft.setPower(0.2);
            backRight.setPower(0.2);
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        arm.runToPositionAsync(Configurable.armHighPosition, 1);
        driveTrain.turn(-90, 0.1, 1);
        driveTrain.driveCM(270, 0.6);
        lowerArm();
    }
}