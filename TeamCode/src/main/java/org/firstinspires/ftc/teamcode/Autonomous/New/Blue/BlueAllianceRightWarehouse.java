package org.firstinspires.ftc.teamcode.Autonomous.New.Blue;

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
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.visionv1.TseDetector;
import org.firstinspires.ftc.teamcode.Configurable;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Blue Right Warehouse", group="FTC22Auto_Ware_Blue")
public class BlueAllianceRightWarehouse extends LinearOpMode {

    TseDetector detector;
    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    Motor arm;
    Motor collector;
    Motor duckSpinner;
    DistanceSensor cargoDetector;
    TouchSensor armTouchSensor;
    BNO055IMU imu;

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
        arm.setPower(1);

        cargoDetector = hardwareMap.get(DistanceSensor.class, "cargoDetector");
        armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouch");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    @Override
    public void runOpMode() {
        initHardware();
        Logger.setTelemetry(telemetry);
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight);

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
        TseDetector.Location itemPos = detector.getLocation(this);
        Logger.addData("Detected Cargo: " + itemPos);
        Logger.update();
        driveTrain.drive(10, 0.8);
        driveTrain.turn(-90, 0.8, imu, AxesOrder.XYZ);
        driveTrain.drive( 57, 0.8);
        driveTrain.turn(0, 0.8, imu, AxesOrder.XYZ);
        switch (itemPos) {
            case LEFT:
                arm.runToPosition(Configurable.armHighPosition, 1);
                break;
            case RIGHT:
                arm.runToPosition(Configurable.armLowPosition, 1);
                break;
            case CENTER:
                arm.runToPosition(Configurable.armMidPosition, 1);
                break;
        }
        driveTrain.drive( 35, 0.8);
        driveTrain.turn(0, 1, imu, AxesOrder.XYZ);
        switch (itemPos) {
            case LEFT:
                collector.setPower(Configurable.disposeLowSpeed);
                break;
            case RIGHT:
                collector.setPower(Configurable.disposeHighSpeed);
                break;
            case CENTER:
                collector.setPower(Configurable.disposeMidSpeed);
                break;
        }
        driveTrain.drive(25, -0.4);
        arm.runToPosition(Configurable.armLowPosition, 1);
        driveTrain.turn(-92, 0.8, imu, AxesOrder.XYZ);
        driveTrain.drive(125, 0.8);
        driveTrain.turn(-90, 0.8, imu, AxesOrder.XYZ);
        duckSpinner.setPower(Configurable.duckSpinnerPower); // 4000 milliseconds
        driveTrain.turn(85, 0.8, imu, AxesOrder.XYZ);
        arm.runToPosition(Configurable.armHighPosition, 1);
        driveTrain.drive(1, 270);
    }
}
