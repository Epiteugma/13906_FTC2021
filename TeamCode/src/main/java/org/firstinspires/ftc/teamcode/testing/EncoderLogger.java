package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.teamcode.autonomous.vision.TseDetector;
import org.firstinspires.ftc.teamcode.Configurable;

@Autonomous(name = "Encoder Logger", group = "SusanaLib")
@Disabled
public class EncoderLogger extends LinearOpMode {

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
        Logger.setTelemetry(telemetry);
        initHardware();
        waitForStart();
        Motor[] motors = new Motor[]{frontLeft, frontRight, backLeft, backRight};
        int[] motorTicks = new int[]{0, 0, 0, 0};
        double[] motorVelo = new double[]{0, 0, 0, 0};
        while (opModeIsActive()) {
            int ticks = 0;
            int averageTicks = 0;
            int averageVelo = 0;
            for(int i = 0; i < motors.length; i++) {
                motorTicks[i] = Math.abs(motors[i].getCurrentPosition());
                motorVelo[i] = Math.abs(motors[i].getVelocity());
                ticks += motorTicks[i];
                averageVelo += motorVelo[i];
            }
            averageVelo /= motors.length;
            averageTicks = ticks / motors.length;

            double ratio = Configurable.driveGearRatio/2;
            double circumference = Configurable.wheelCircumference;

            double cm = (circumference * averageTicks) / (28 * ratio);

            Logger.addData("Average ticks: " + averageTicks);
            Logger.addData("CM: " + cm);
            Logger.addData("Velo: " + averageVelo);
            Logger.update();
        }
    }

}
