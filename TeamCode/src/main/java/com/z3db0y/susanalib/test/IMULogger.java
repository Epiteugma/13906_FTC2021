package com.z3db0y.susanalib.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "IMU Logger", group = "SusanaLib")
public class IMULogger extends LinearOpMode {
    BNO055IMU imu;

    private void initHardware() {

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
        waitForStart();
        while (opModeIsActive()) {
            Logger.addData(imu.getAngularOrientation().firstAngle);
            Logger.addData(imu.getAngularOrientation().secondAngle);
            Logger.addData(imu.getAngularOrientation().thirdAngle);
            Logger.update();
        }
    }

}
