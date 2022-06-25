package com.z3db0y.susanalib;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumDriveTrain {
    Motor[] motors;
    public double turnThreshold = 4;
    public double ratio = 20;
    public double wheelRadius = 3.75;
    BNO055IMU imu;
    double lastStallCheck = 0;
    double lastVelo = 0;

    private void init(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, BNO055IMU imu) {
        this.motors = new Motor[]{frontLeft, frontRight, backLeft, backRight};
        for(Motor motor : motors) {
            motor.resetEncoder();
        }
        this.imu = imu;
    }

    public MecanumDriveTrain(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, BNO055IMU imu) {
        // Default wheel radius = Mecanum wheels
        // Default gear ratio 20:1
        init(frontLeft, frontRight, backLeft, backRight, imu);
    }

    public void resetStallDetector() {
        lastStallCheck = 0;
        lastVelo = 0;
    }

    public boolean isStalled() {
        if(lastStallCheck == 0) lastStallCheck = System.currentTimeMillis();
        double velo = 0;
        for(Motor motor : motors) {
            velo += Math.abs(motor.getVelocity());
        }
        velo /= motors.length;
        Logger.addData("Velo: " + velo);
        Logger.addData("Last velo: " + lastVelo);
        boolean stalled = false;
        if(System.currentTimeMillis() - lastStallCheck > 1000) {
            if(Math.abs(velo - lastVelo) == 0) {
                stalled = true;
            }
            lastVelo = velo;
        }
        return stalled;
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
        lastStallCheck = 0;
        release();
        resetEncoders();
        runOnEncoders();

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
        }
        while(
                (Math.abs(motors[0].getCurrentPosition()) < Math.abs(targetPositions[0]) ||
                Math.abs(motors[1].getCurrentPosition()) < Math.abs(targetPositions[1]) ||
                Math.abs(motors[2].getCurrentPosition()) < Math.abs(targetPositions[2]) ||
                Math.abs(motors[3].getCurrentPosition()) < Math.abs(targetPositions[3])) &&
                        !isStalled()
        ) {
            Logger.addData("Front left: " + motors[0].getCurrentPosition() + " / " + motors[0].getTargetPosition());
            Logger.addData("Front right: " + motors[1].getCurrentPosition() + " / " + motors[1].getTargetPosition());
            Logger.addData("Back left: " + motors[2].getCurrentPosition() + " / " + motors[2].getTargetPosition());
            Logger.addData("Back right: " + motors[3].getCurrentPosition() + " / " + motors[3].getTargetPosition());
            Logger.update();
        }

        hold();
    }

    private double normalizeAngle(double angle) {
        if(angle < -180) return -180;
        if(angle > 180) return 180;
        return angle;
    }

    public void resetEncoders() {
        for(Motor motor : motors) {
            motor.resetEncoder();
        }
    }

    public void driveCM(int cm, double power) {
        this.drive((int) ((cm / (wheelRadius * Math.PI * 2)) * (28 * ratio)), power);
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

        release();
        resetEncoders();
        runOnEncoders();

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

        hold();
    }

    private double getCurrentAngle(int angle) {
        Orientation angles = imu.getAngularOrientation();
        switch (angle) {
            case 1: return angles.firstAngle;
            case 2: return angles.secondAngle;
            case 3: return angles.thirdAngle;
        }
        return 0;
    }

    public void runOnEncoders(){
        for(Motor motor : motors) {
            motor.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void hold(){
        for(Motor motor : motors) {
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }
    }

    public void release() {
        for(Motor motor : motors) {
            motor.setPower(0);
        }
    }

    public void turn(double targetAngle, double power, int angle) {

        release();
        resetEncoders();
        runOnEncoders();

        double currentAngle;
        if(targetAngle > 180) targetAngle -= 360;

        double rangeMin = normalizeAngle(targetAngle + turnThreshold/2);
        double rangeMax = normalizeAngle(targetAngle - turnThreshold/2);

        double diff;
        do {
            currentAngle = getCurrentAngle(angle);
            if(currentAngle == 180 && targetAngle < 0) currentAngle = -180;

            double target360 = targetAngle < 0 ? targetAngle + 360 : targetAngle;
            double current360 = currentAngle < 0 ? currentAngle + 360 : currentAngle;
            double diff360 = current360 - target360;
            int directionMultiplier = Math.abs(diff360) > 180 ? (diff360 > 0 ? 1 : -1) : (diff360 > 0 ? -1 : 1);

            diff = Math.abs(currentAngle - targetAngle);
            Logger.addData("Tar: " + targetAngle + " / Curr: " + currentAngle + " / Diff: " + (targetAngle - currentAngle));
            Logger.addData("Dir: " + directionMultiplier);
            Logger.addData("Range Min: " + rangeMin);
            Logger.addData("Range Max: " + rangeMax);
            for(int i = 0; i < motors.length; i++) {
                if(i % 2 == 0) motors[i].setPower(power * directionMultiplier);
                else motors[i].setPower(-power * directionMultiplier);
            }
            Logger.update();
        } while(diff > turnThreshold);

        hold();
    }
}
