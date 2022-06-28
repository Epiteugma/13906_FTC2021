package org.firstinspires.ftc.teamcode.autonomous.opmodes.blue;

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
import org.firstinspires.ftc.teamcode.Configurable;
import org.firstinspires.ftc.teamcode.autonomous.vision.TseDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue Left", group = "FTC22Auto")
public class Left extends LinearOpMode {
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

    int timer = 30;

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
        driveTrain.setPower(-power, -power, -power, -power);
        collector.setPower(-1);
        double currentAngle = imu.getAngularOrientation().firstAngle;
        do {
            Logger.addData("Initial Distance: " + initialDistance);
            Logger.addData("Distance: " + cargoDetector.getDistance(DistanceUnit.CM));
            Logger.update();
            if (initialDistance - cargoDetector.getDistance(DistanceUnit.CM) < 2) {
                driveTrain.driveCM(25, 0.2);
            }
            if (initialDistance - cargoDetector.getDistance(DistanceUnit.CM) < 2) {
                driveTrain.turn(currentAngle - 8, 0.1, 1);
            }
            if (initialDistance - cargoDetector.getDistance(DistanceUnit.CM) < 2) {
                driveTrain.turn(currentAngle + 8, 0.1, 1);
            }
        } while (initialDistance - cargoDetector.getDistance(DistanceUnit.CM) < 2);
        collector.setPower(0);
        driveTrain.hold();

        // Go back to initial position.
        driveTrain.drive((int) (-averageTicks * 0.2), 0.2);
    }

    private void releaseCube(double collectorPower) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < startTime + 1000) {
            double percentage = (System.currentTimeMillis() - startTime) / 1000;
            collector.setPower(collectorPower * percentage);
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

    private void driveBackWallDistance(double distance) {
        driveTrain.runOnEncoders();
        while (backDistance.getDistance(DistanceUnit.CM) > distance) {
            Logger.addData("Distance: " + backDistance.getDistance(DistanceUnit.CM));
            Logger.update();
            driveTrain.setPowerAll(0.2);
        }
        driveTrain.hold();
    }

    private void startTimer() {
        new Thread(() -> {
            double prevTime = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() > prevTime + 1000) {
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
        TseDetector.Location itemPos = detector.getLocation();
        if (itemPos != null) {
            Logger.addData("Detected Cargo: " + itemPos);
        } else {
            itemPos = TseDetector.Location.CENTER;
            Logger.addData("Fallback to CENTER");
        }
        Logger.update();

        driveTrain.driveCM(15, 0.4);
        driveTrain.turn(-90, 0.1, 1);
        driveTrain.driveCM(75, 0.2);
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
                releaseCube(Configurable.disposeHighSpeed);
                break;
            case CENTER:
                arm.runToPositionAsync(Configurable.armMidPosition, 1);
                driveToShippingHub(0.2, Configurable.distanceToShippingHubBlueLow);
                releaseCube(Configurable.disposeMidSpeed);
                break;
        }
        driveTrain.driveCM(-15, 0.3);
        arm.runToPositionAsync(Configurable.armHighPosition, 1);
        driveTrain.turn(90, 0.1, 1);
        driveTrain.driveCM(250, 0.6);
        lowerArm();
        driveTrain.turn(105, 0.1, 1);
        collectCube(0.2);
        if (timer < 5) {
            this.stop();
        }
        arm.runToPositionAsync(Configurable.armHighPosition, 1);
        driveTrain.turn(-90, 0.1, 1);
        driveTrain.driveCM(300, 0.6);
        driveTrain.turn(0, 0.1, 1);
        driveToShippingHub(0.2, Configurable.distanceToShippingHubBlueHigh);
        releaseCube(Configurable.disposeLowSpeed);
        driveTrain.runOnEncoders();
        driveBackWallDistance(Configurable.distancefromBackWallBlue);
        arm.runToPositionAsync(Configurable.armHighPosition, 1);
        driveTrain.turn(90, 0.1, 1);
        driveTrain.driveCM(270, 0.6);
        lowerArm();
    }
}
