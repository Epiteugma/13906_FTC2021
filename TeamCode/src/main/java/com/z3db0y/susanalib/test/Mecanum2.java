package com.z3db0y.susanalib.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@TeleOp(name = "Mecanum Field Centric Test", group = "SusanaLib")
public class Mecanum2 extends LinearOpMode {

    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    BNO055IMU imu;

    private void initHardware() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {
        initHardware();

        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight, imu);

        waitForStart();
        while (opModeIsActive()) {
            driveTrain.driveFieldCentric(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        }
    }

}
