package com.z3db0y.susanalib;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Arrays;

public class MecanumDriveTrain {
    Motor[] motors;

    private void init(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        this.motors = new Motor[]{frontLeft, frontRight, backLeft, backRight};
    }

    public MecanumDriveTrain(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        init(frontLeft, frontRight, backLeft, backRight);
    }

    public void driveRobotCentric(double forwardPower, double sidePower, double strafePower) {
        motors[0].setPower(forwardPower - sidePower - strafePower*0.9); // front left
        motors[1].setPower(forwardPower + sidePower + strafePower*0.9); // front right
        motors[2].setPower(forwardPower - sidePower + strafePower); // back left
        motors[3].setPower(forwardPower + sidePower - strafePower); // back right
    }

    public void driveFieldCentric(double forwardPower, double sidePower, double strafePower, double robotAngle) {
//        double relativeAngle = robotAngle - Math.atan2(-forwardPower, strafePower);
//        double gamepadHypot = Range.clip(Math.hypot(strafePower, forwardPower), 0, 1);
//
//        double xValue = Math.cos(Math.toRadians(relativeAngle)) * gamepadHypot;
//        double yValue = Math.sin(Math.toRadians(relativeAngle)) * gamepadHypot;
//
//        double relativeForwardPower = yValue * Math.abs(yValue);
//        double relativeStrafePower = xValue * Math.abs(xValue);
//
//        motors[0].setPower(relativeForwardPower - sidePower - relativeStrafePower);
//        motors[1].setPower(relativeForwardPower + sidePower + relativeStrafePower);
//        motors[2].setPower(relativeForwardPower - sidePower + relativeStrafePower);
//        motors[3].setPower(relativeForwardPower + sidePower - relativeStrafePower);

        throw new NotImplementedError();
    }

    public void drive(int relativeTicks, double power) {
        int[] targetPositions = new int[]{
                motors[0].getCurrentPosition()-(relativeTicks * motors[0].getDirection().getMultiplier()),
                motors[1].getCurrentPosition()-(relativeTicks * motors[1].getDirection().getMultiplier()),
                motors[2].getCurrentPosition()-(relativeTicks * motors[2].getDirection().getMultiplier()),
                motors[3].getCurrentPosition()-(relativeTicks * motors[3].getDirection().getMultiplier())
        };

        for(int i = 0; i < targetPositions.length; i++) {
            Motor motor = motors[i];
            motor.setTargetPosition(targetPositions[i]);
            motor.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
            Logger.addData(motor.getDirection());
            Logger.addData(motor.getTargetPosition());
            Logger.addData(motor.getPower());
        }
        Logger.update();
        while(
                Math.abs(motors[0].getCurrentPosition()) < Math.abs(targetPositions[0]) ||
                Math.abs(motors[1].getCurrentPosition()) < Math.abs(targetPositions[1]) ||
                Math.abs(motors[2].getCurrentPosition()) < Math.abs(targetPositions[2]) ||
                Math.abs(motors[3].getCurrentPosition()) < Math.abs(targetPositions[3])
        ) {}
        for (Motor motor : motors) {
            motor.setHoldPosition(true);
            motor.setPower(0);
            motor.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public enum Side {
        LEFT(1), RIGHT(-1);

        private final int multiplier;
        Side(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    public void strafe(Side side, int relativeTicks, double power) {
        int[] targetPositions = new int[]{
                motors[0].getCurrentPosition() + side.getMultiplier() * (relativeTicks * motors[0].getDirection().getMultiplier()),
                motors[1].getCurrentPosition() - side.getMultiplier() * (relativeTicks * motors[1].getDirection().getMultiplier()),
                motors[2].getCurrentPosition() - side.getMultiplier() * (relativeTicks * motors[2].getDirection().getMultiplier()),
                motors[3].getCurrentPosition() + side.getMultiplier() * (relativeTicks * motors[3].getDirection().getMultiplier())
        };
        for(int i = 0; i < targetPositions.length; i++) {
            Motor motor = motors[i];
            motor.setTargetPosition(targetPositions[i]);
            motor.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        while(
                Math.abs(motors[0].getCurrentPosition()) < Math.abs(targetPositions[0]) ||
                        Math.abs(motors[1].getCurrentPosition()) < Math.abs(targetPositions[1]) ||
                        Math.abs(motors[2].getCurrentPosition()) < Math.abs(targetPositions[2]) ||
                        Math.abs(motors[3].getCurrentPosition()) < Math.abs(targetPositions[3])
        ) {}
        for (Motor motor : motors) {
            motor.setHoldPosition(true);
            motor.setPower(0);
            motor.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getCurrentAngle(BNO055IMU imu, AxesOrder order) {
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, order, AngleUnit.DEGREES).firstAngle;
        if(currentAngle < 0) currentAngle = 360 + currentAngle;
        Logger.addData(currentAngle);
        Logger.update();
        return currentAngle;
    }

    public void turn(double targetAngle, double power, BNO055IMU imu, AxesOrder order) {
        double threshold = 2;
        int directionMultiplier = targetAngle - getCurrentAngle(imu, order) > 0 ? 1 : -1;

        for(int i = 0; i < motors.length; i++) {
            if(i % 2 == 0) motors[i].setPower(power * directionMultiplier);
            else motors[i].setPower(-power * directionMultiplier);
        }

        while (
                getCurrentAngle(imu, order) < targetAngle - threshold/2 ||
                getCurrentAngle(imu, order) > targetAngle + threshold/2
        ) {}

        for(Motor motor : motors) {
            motor.setHoldPosition(true);
            motor.setPower(0);
        }
    }
}
