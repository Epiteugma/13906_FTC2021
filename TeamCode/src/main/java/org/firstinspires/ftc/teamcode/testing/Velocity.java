package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import org.firstinspires.ftc.teamcode.Configurable;

@TeleOp(name = "1Ad-dA0", group = "SusanaLib")
public class Velocity extends LinearOpMode {
    double lastVelo = 0;
    double lastTicks = 0;
    double lastCheck = 0;

    private boolean isStalled(DcMotorEx motor) {
        double velo = motor.getVelocity();
        double ticks = motor.getCurrentPosition();
        double delta = Math.abs(lastVelo - velo);
        double tickDelta = Math.abs(lastTicks - ticks);
        if(lastCheck+1000 > System.currentTimeMillis()) return false;
        lastCheck = System.currentTimeMillis();
        lastVelo = velo;
        if(delta < 20 && tickDelta < 28 && motor.getPower() != 0)  {
            return true;
        }
        return false;
    }

    @Override
    public void runOpMode() {
        waitForStart();
//        DcMotor coreHex = hardwareMap.get(DcMotor.class, "test");
//        coreHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        coreHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
//        coreHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while (opModeIsActive() && !isStalled((DcMotorEx) coreHex)) {
//        while (opModeIsActive()) {
////            coreHex.setPower(0.5);
//            Logger.setTelemetry(telemetry);
//            Logger.addData("Last velocity: " + lastVelo);
//            Logger.addData("Velocity: " + ((DcMotorEx)coreHex).getVelocity());
//            Logger.addData("Ticks: " + coreHex.getCurrentPosition());
//            Logger.update();
//        }
        Motor backLeft = new Motor(hardwareMap, "backLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor frontRight = new Motor(hardwareMap, "frontRight");

        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight, imu);

        driveTrain.strafeCM(MecanumDriveTrain.Side.RIGHT, 10, 0.2);
    }
}
