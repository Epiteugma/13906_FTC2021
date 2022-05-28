package com.z3db0y.susanalib;

import com.qualcomm.robotcore.util.Range;

public class MecanumDriveTrain {
    Motor[] motors;

    private void init(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        this.motors = new Motor[]{frontLeft, frontRight, backLeft, backRight};
    }

    public MecanumDriveTrain(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        init(frontLeft, frontRight, backLeft, backRight);
    }

    public void driveRobotCentric(double forwardPower, double sidePower, double strafePower) {
        motors[0].setPower(forwardPower - sidePower - strafePower); // front left
        motors[1].setPower(forwardPower + sidePower + strafePower); // front right
        motors[2].setPower(forwardPower - sidePower + strafePower); // back left
        motors[3].setPower(forwardPower + sidePower - strafePower); // back right
    }

    public void driveFieldCentric(double forwardPower, double sidePower, double strafePower, double robotAngle) {
        double relativeAngle = robotAngle - Math.atan2(-forwardPower, strafePower);
        double gamepadHypot = Range.clip(Math.hypot(strafePower, forwardPower), 0, 1);

        double xValue = Math.cos(Math.toRadians(relativeAngle)) * gamepadHypot;
        double yValue = Math.sin(Math.toRadians(relativeAngle)) * gamepadHypot;

        double relativeForwardPower = yValue * Math.abs(yValue);
        double relativeStrafePower = xValue * Math.abs(xValue);

        motors[0].setPower(relativeForwardPower - sidePower - relativeStrafePower);
        motors[1].setPower(relativeForwardPower + sidePower + relativeStrafePower);
        motors[2].setPower(relativeForwardPower - sidePower + relativeStrafePower);
        motors[3].setPower(relativeForwardPower + sidePower - relativeStrafePower);

//        throw new NotImplementedError();
    }
}
