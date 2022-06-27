package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "TeleOp", group = "FTC22")
public class Drive extends LinearOpMode {
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

    public void initHardware() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");
        arm = new Motor(hardwareMap, "arm");
        collector = new Motor(hardwareMap, "collector");
        duckSpinner = new Motor(hardwareMap, "duckSpinner");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Motor reversing
        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        frontLeft.setHoldPosition(true);
//        frontRight.setHoldPosition(true);
//        backLeft.setHoldPosition(true);
//        backRight.setHoldPosition(true);

        arm.setTargetPosition(0);
        arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        cargoDetector = hardwareMap.get(DistanceSensor.class, "cargoDetector");
        armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouch");
        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");
    }

    double duckSpinnerPower = Configurable.duckSpinnerPower;
    double globalPowerFactor = 0.7;
    boolean duckSpinnersActivated = false;
    double lastResetTime = 0;
    long prevTime = 0;

    private void duckSpinnerControl() {
        if (gamepad1.dpad_up && System.currentTimeMillis() >= prevTime + 200 && duckSpinnerPower + 0.05 < 1) {
            prevTime = System.currentTimeMillis();
            duckSpinnerPower += 0.05;
        } else if (gamepad1.dpad_down && System.currentTimeMillis() >= prevTime + 200 && duckSpinnerPower -0.05 > 0) {
            prevTime = System.currentTimeMillis();
            duckSpinnerPower -= 0.05;
        }
        else if (gamepad1.dpad_right && System.currentTimeMillis() >= prevTime + 200) {
            prevTime = System.currentTimeMillis();
            duckSpinnerPower = -duckSpinnerPower;
        }

        if (duckSpinnersActivated) {
            duckSpinner.setPower(duckSpinnerPower);
        }

        if (gamepad1.b && System.currentTimeMillis() >= prevTime + 300) {
            prevTime = System.currentTimeMillis();
            if (duckSpinnersActivated) {
                duckSpinnersActivated = false;
                duckSpinner.setPower(0);
            } else {
                duckSpinnersActivated = true;
                duckSpinner.setPower(duckSpinnerPower);
            }
            duckSpinner.setHoldPosition(!duckSpinnersActivated);
        }
    }

    private void armControl() {
        if (gamepad2.right_bumper) {
            arm.setTargetPosition(-1750);
        }
        else if (gamepad2.left_bumper && !armTouchSensor.isPressed()) {
            arm.setTargetPosition(2500);
        }
        else if (gamepad2.y) {
            arm.setTargetPosition(-1000);
        }
        else if (gamepad2.a) {
            arm.setTargetPosition(-370);
        }

        if (armTouchSensor.isPressed() && lastResetTime + 250 < System.currentTimeMillis()) {
            arm.resetEncoder();
            arm.setTargetPosition(0);
            arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            Logger.addData("touchSensor is currently pressed and I am ready to collect!", "#ef3f49");
            lastResetTime = System.currentTimeMillis();
        }
        else {
            Logger.addData("ArmReset: " + "False");
        }

        if (gamepad2.dpad_down && !armTouchSensor.isPressed() && arm.getTargetPosition() != 0) {
            arm.setTargetPosition(arm.getCurrentPosition() + 150);
        }
        else if (gamepad2.dpad_up) {
            arm.setTargetPosition(arm.getCurrentPosition() - 150);
        }

        if(Math.abs(gamepad2.left_stick_y) != 0)
        arm.setTargetPosition(arm.getCurrentPosition() + (int)(gamepad2.left_stick_y * 150));

        arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void globalPowerFactorControl() {
        if (gamepad1.left_bumper && globalPowerFactor - 0.1 > 0 && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor -= 0.1;
        } else if (gamepad1.right_bumper && globalPowerFactor + 0.1 < 0.7 && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor += 0.1;
        }

        if (gamepad1.triangle && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            if (Math.abs(globalPowerFactor - 0.3) < Math.abs(globalPowerFactor - 0.6)) {
                globalPowerFactor = 0.6;
            } else {
                globalPowerFactor = 0.3;
            }
        }
        if (gamepad1.cross && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor = 1;
        }
    }

    private void collectorControl() {
        if (gamepad2.right_trigger > 0) {
            collector.setPower(gamepad2.right_trigger * 0.8);
        }
        else if (gamepad2.left_trigger > 0) {
            collector.setPower(-gamepad2.left_trigger);
        }
        else collector.setPower(0);
    }

    private void Logging() {
        Logger.addData("Powers:");
        Logger.addData("|--  GlobalPowerFactor: " + globalPowerFactor);
        Logger.addData("|--  frontRight power: " + frontRight.getPower());
        Logger.addData("|--  frontLeft power: " + frontLeft.getPower());
        Logger.addData("|--  backRight power: " + backRight.getPower());
        Logger.addData("|--  backLeft power: " + backLeft.getPower());
        Logger.addData("|--  Arm power: " + arm.getPower());
        Logger.addData("|--  Collector power: " + collector.getPower());
        Logger.addData("Ticks:");
        Logger.addData("|--  Current arm ticks: " + arm.getCurrentPosition());
        Logger.addData("|--  Target arm ticks: " + arm.getTargetPosition());
        Logger.addData("|--  ducksSpinner power: " + duckSpinner.getPower());
        Logger.addData("|--  ducksSpinner variable: " + duckSpinnerPower);
        Logger.addData("|--  frontRight ticks: " + frontRight.getCurrentPosition());
        Logger.addData("|--  frontLeft ticks: " + frontLeft.getCurrentPosition());
        Logger.addData("|--  backRight ticks: " + backRight.getCurrentPosition());
        Logger.addData("|--  backLeft ticks: " + backLeft.getCurrentPosition());
        Logger.update();
    }

    @Override
    public void runOpMode() {

//        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(webcam, 30);
//            }
//
//            @Override
//            public void onError(int code) {
//
//            }
//        });

        initHardware();
        Logger.setTelemetry(telemetry);

        double distance = cargoDetector.getDistance(DistanceUnit.CM);
        double prevDistance = distance;

        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight, imu);
        waitForStart();
        while (opModeIsActive()) {
            distance = cargoDetector.getDistance(DistanceUnit.CM);
            Logger.addData("PrevDistance: " + prevDistance);
            Logger.addData("Distance: " + distance);

            if (Math.abs(prevDistance - distance) >= 2) {
                gamepad1.rumble(1, 1, 200);
                gamepad2.rumble(1, 1, 200);
                prevDistance = distance;
            }

            if (driveTrain.isStalled()) {
                Logger.addData("Drivetrain is stalled");
                gamepad1.rumble(0.5, 1, 1000);
                gamepad2.rumble(0.5, 1, 1000);
            }

            if(gamepad1.right_trigger > 0) {
                gamepad2.rumble(gamepad1.right_trigger, gamepad1.right_trigger, 1);
            }

            duckSpinnerControl();

            globalPowerFactorControl();

            driveTrain.driveRobotCentric(gamepad1.left_stick_y * globalPowerFactor, gamepad1.right_stick_x, gamepad1.left_stick_x * globalPowerFactor);

            armControl();

            collectorControl();

            Logging();
        }
    }

}
