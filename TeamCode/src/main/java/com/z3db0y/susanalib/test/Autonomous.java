package com.z3db0y.susanalib.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous Test", group = "SusanaLib")
public class Autonomous extends LinearOpMode {
    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    BNO055IMU imu;

    private void initMotors() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        // Motor reversing
//        frontLeft.setDirection(Motor.Direction.REVERSE);
        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);
//        backRight.setDirection(Motor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    @Override
    public void runOpMode() {
        initMotors();
        Logger.setTelemetry(telemetry);
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight);
        waitForStart();

//        driveTrain.drive(500, 0.5);
        driveTrain.turn(90,0.5, imu, AxesOrder.XYZ);
//        driveTrain.strafe(MecanumDriveTrain.Side.RIGHT, 500, 0.5);
    }

}
