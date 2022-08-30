package org.firstinspires.ftc.teamcode.ftc2022.autonomous.opmodes.blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc2022.Configurable;
import org.firstinspires.ftc.teamcode.ftc2022.autonomous.vision.TseDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

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

        cargoDetector = hardwareMap.get(DistanceSensor.class, "cargoDetector");
        armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouch");
        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    private void releaseCube(double collectorPower) {
        collector.setPower(collectorPower);
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < startTime + 1000) {
            double percentage = (System.currentTimeMillis() - startTime) / 1000;
            double addition = 1 - collectorPower;
            collector.setPower(collectorPower + addition * percentage);
        }
        collector.setPower(0);
    }

    private void driveToShippingHub(double power, double distance) {
        while (backDistance.getDistance(DistanceUnit.CM) < distance) {
            driveTrain.setPowerAll(-power);
        }
        driveTrain.turn(0, 0.1, 1);
        driveTrain.hold();
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

    private void driveBackWallDistance(double distance) {
        while (backDistance.getDistance(DistanceUnit.CM) > distance) {
            Logger.addData("Distance: " + backDistance.getDistance(DistanceUnit.CM));
            Logger.update();
            driveTrain.setPowerAll(0.2);
        }
        driveTrain.hold();
    }

    @Override
    public void runOpMode() throws InterruptedException {
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
        TseDetector.Location itemPos = detector.getLocation();
        if (itemPos != null) {
            Logger.addData("Detected Cargo: " + itemPos);
            Logger.update();
        }
        else {
            itemPos = TseDetector.Location.CENTER;
        }

        driveTrain.driveCM(15, 0.4);
        driveTrain.turn(91, 0.1, 1);
        driveTrain.driveCM(76, 0.4);
        driveTrain.turn(0, 0.1, 1);
        switch (itemPos) {
            case LEFT:
                arm.runToPositionAsync(Configurable.armLowPosition, 1);
                driveToShippingHub(0.2, Configurable.distanceToShippingHubBlueLow);
                releaseCube(Configurable.disposeLowSpeed);
                break;
            case RIGHT:
                arm.runToPositionAsync(Configurable.armHighPosition, 1);
                driveToShippingHub(0.2, Configurable.distanceToShippingHubBlueHigh);
                releaseCube(Configurable.disposeMidSpeed);
                break;
            case CENTER:
                arm.runToPositionAsync(Configurable.armMidPosition, 1);
                driveToShippingHub(0.2, Configurable.distanceToShippingHubBlueMid);
                releaseCube(Configurable.disposeLowSpeed);
                break;
        }
        driveBackWallDistance(Configurable.distanceFromBackWallBlue);
        lowerArmAsync();
        driveTrain.turn(-90, 0.1, 1);
        driveTrain.driveCM(65, 0.3);
        driveTrain.turn(-90, 0.1, 1);
        driveTrain.driveCM(65, 0.1);

        com.z3db0y.susanalib.Thread.sleep(300);

        double duckSpinnerPower = Configurable.duckSpinnerPowerBlue;
        duckSpinner.resetEncoder();
        duckSpinner.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckSpinner.resetStallDetection();
        while (Math.abs(duckSpinner.getCurrentPosition()) - Math.abs(Configurable.duckSpinnerTicks) < 0) {
            if (duckSpinner.isStalled()) {
                duckSpinnerPower += 0.05;
                Logger.addData("Duck Spinner Power: " + duckSpinnerPower);
            }
            if(!driveTrain.isStalled()) {
                driveTrain.resetStallDetector();
                driveTrain.strafeCM(MecanumDriveTrain.Side.RIGHT ,5, 0.1);
            }
            Logger.addData(duckSpinner.getCurrentPosition());
            Logger.update();
            duckSpinner.setPower(-duckSpinnerPower);
        }
        duckSpinner.setPower(0);
    }
}