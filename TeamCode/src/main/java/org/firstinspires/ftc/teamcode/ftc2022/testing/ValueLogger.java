package org.firstinspires.ftc.teamcode.ftc2022.testing;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.Motor;

@TeleOp(name = "Value Logger", group = "FTC22")
@Disabled
public class ValueLogger extends LinearOpMode {

    BNO055IMU imu;
    Motor core_hex;
    Motor _20_1;
    Motor _40_1;
    Motor ultraplanetary;


    private void initHardware() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        core_hex = new Motor(hardwareMap, "core_hex");
        _20_1 = new Motor(hardwareMap, "_20_1");
        _40_1 = new Motor(hardwareMap, "_40_1");
        ultraplanetary = new Motor(hardwareMap, "ultraplanetary");
    }

    @Override
    public void runOpMode() {
        Logger.setTelemetry(telemetry);
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            Logger.addData("Core Hex: " + core_hex.getCurrentPosition());
            Logger.addData("20_1: " + _20_1.getCurrentPosition());
            Logger.addData("40_1: " + _40_1.getCurrentPosition());
            Logger.addData("ultraplanetary: " + ultraplanetary.getCurrentPosition());
            Logger.update();
        }
    }
}
