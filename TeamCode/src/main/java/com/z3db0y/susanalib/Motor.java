package com.z3db0y.susanalib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    private final DcMotor motor;
    private Direction direction;
    private boolean holdPosition;
    private double lastStallCheck = 0;
    private double lastVelo;
    private double power = 0;
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public double wheelRadius = 3.75;
    public double ratio = 20;

    public enum Direction {
        FORWARD(1), REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() { return this.multiplier; }
    }

    public Motor(HardwareMap hardwareMap, String deviceName) {
        this.motor = hardwareMap.get(DcMotor.class, deviceName);
        this.direction = Direction.FORWARD;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(this.runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        this.motor.setZeroPowerBehavior(behavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() { return this.motor.getZeroPowerBehavior(); }

    public void setDirection(Direction dir) {
        this.direction = dir;
    }

    public Direction getDirection() {
        return this.direction;
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        this.runMode = runMode;
    }

    public DcMotor.RunMode getRunMode() {
        return this.runMode;
    }

    public void setPower(double power) {
        this.motor.setMode(this.runMode);
        this.power = power;
        this.motor.setPower(power * this.direction.getMultiplier());
        if(this.getPower() == 0 && this.holdPosition) {
            this.motor.setTargetPosition(this.motor.getCurrentPosition());
            this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.motor.setPower(1);
        }
    }

    public double getPower() {
        return this.power;
    }

    public void setHoldPosition(boolean hold) {
        this.holdPosition = hold;
    }

    public boolean getHoldPosition() {
        return this.holdPosition;
    }

    public int getPosition() {
        return this.motor.getCurrentPosition();
    }

    // Only to be used in runToPosition function - because lastStallCheck has to be reset.
    private boolean isStalled() {
        if(lastStallCheck == 0) lastStallCheck = System.currentTimeMillis();
        double velo = this.getVelocity();
        Logger.addData("Velo: " + velo);
        boolean stalled = false;
        if(System.currentTimeMillis() - lastStallCheck > 1000) {
            double delta = velo - lastVelo;
            if(delta == 0) stalled = true;
            Logger.addData("Delta: " + delta);
            lastVelo = velo;
        }
        return stalled;
    }

    public boolean runToPosition(int ticks, double power) {
        this.lastStallCheck = 0;
        this.motor.setTargetPosition(this.direction.getMultiplier() * ticks);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setPower(power);

        boolean wasStalled = false;
        while(Math.abs(this.getPosition()) < Math.abs(ticks) && !(wasStalled = isStalled())) {
            Logger.addData(this.getPosition());
        }
        this.setPower(0);
        return wasStalled;
    }

    public void runToPositionAsync(int ticks, double power) {
        this.motor.setTargetPosition(ticks);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setPower(power);
    }

    public void setTargetPosition(int ticks) {
        this.motor.setTargetPosition(ticks);
    }

    public int getTargetPosition() {
        return this.motor.getTargetPosition();
    }

    public int getCurrentPosition() {
        return this.motor.getCurrentPosition();
    }

    public double getVelocity() {
        return ((DcMotorEx)this.motor).getVelocity();
    }

    public int calculateTicks(int cm) {
        double targetRotations = cm / (2 * Math.PI * wheelRadius);
        return (int) (targetRotations * (28 * ratio));
    }
}
