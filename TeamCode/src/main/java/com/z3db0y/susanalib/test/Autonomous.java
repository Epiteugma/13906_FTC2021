package com.z3db0y.susanalib.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Configurable;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Test", group = "SusanaLib")
public class Autonomous extends LinearOpMode {
    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    BNO055IMU imu;
    Motor collector;
    Motor arm;
    DistanceSensor cargoDetector;
    MecanumDriveTrain driveTrain;

    private void initHardware() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        collector = new Motor(hardwareMap, "collector");
        arm = new Motor(hardwareMap, "arm");
        cargoDetector = hardwareMap.get(DistanceSensor.class, "cargoDetector");

        // Motor reversing
//        frontLeft.setDirection(Motor.Direction.REVERSE);
        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);
//        backRight.setDirection(Motor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight, imu);
    }

    private void collectCube(double power) {
        double averageTicks = Math.abs(frontLeft.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()) / 4;
        frontLeft.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);
        collector.setPower(-1);
        double initialDistance = cargoDetector.getDistance(DistanceUnit.CM);
        while (initialDistance - cargoDetector.getDistance(DistanceUnit.CM) < 2) {}
        collector.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Go back to initial position.
        driveTrain.drive((int)averageTicks, 0.3);
    }

    @Override
    public void runOpMode() {
        initHardware();
        Logger.setTelemetry(telemetry);
        waitForStart();

        driveTrain.turn(130, 0.1, 1);
        collectCube(0.2);
    }

}
