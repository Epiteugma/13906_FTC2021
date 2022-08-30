package org.firstinspires.ftc.teamcode.ftc2022.testing;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.z3db0y.susanalib.Logger;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "IMU Logger", group = "SusanaLib")
@Disabled
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
        waitForStart();
        Logger.setTelemetry(telemetry);
        while (opModeIsActive()) {
            Logger.addData(imu.getAngularOrientation().firstAngle);
            Logger.addData(imu.getAngularOrientation().secondAngle);
            Logger.addData(imu.getAngularOrientation().thirdAngle);
            Logger.addData(imu.getLinearAcceleration().xAccel);
            Logger.addData(imu.getLinearAcceleration().yAccel);
            Logger.addData(imu.getLinearAcceleration().zAccel);

            Logger.update();
        }
    }

}
