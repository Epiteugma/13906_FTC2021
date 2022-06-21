package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Beta drive", group = "FTC22")
public class NewDriveMecanum extends LinearOpMode {
    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    Motor arm;
    Motor collector;
    Motor duckSpinner;
    DistanceSensor distanceSensor;
    TouchSensor armTouchSensor;

    public void initMotors() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");
        arm = new Motor(hardwareMap, "arm");
        collector = new Motor(hardwareMap, "collector");

        duckSpinner = new Motor(hardwareMap, "duckSpinner");

        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setTargetPosition(0);
        arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "cargoDetector");
        armTouchSensor = hardwareMap.get(TouchSensor.class, "armTouch");
    }

    @Override
    public void runOpMode() {

        Logger.setTelemetry(telemetry);

        double duckSpinnerPower = 0.25;
        double globalPowerFactor = 0.65;
        boolean lastTouching = false;
        boolean duckSpinnersActivated = false;
        double lastResetTime = 0;
        long prevTime = 0;
        initMotors();
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        double prevDistance = distance;
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight);
        waitForStart();
        while(opModeIsActive()) {
            distance = distanceSensor.getDistance(DistanceUnit.CM);
            Logger.addData("PrevDistance: " + prevDistance);
            Logger.addData("Distance: " + distance);
            if(Math.abs(prevDistance - distance) >= 2) {
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
                prevDistance = distance;
            }

            if (gamepad1.dpad_up && System.currentTimeMillis() >= prevTime + 200 && duckSpinnerPower > 0) {
                prevTime = System.currentTimeMillis();
                duckSpinnerPower += 0.05;
            }
            else if (gamepad1.dpad_down && System.currentTimeMillis() >= prevTime + 200 && duckSpinnerPower < 1){
                prevTime = System.currentTimeMillis();
                duckSpinnerPower -= 0.05;
            }


            else if (gamepad1.dpad_right && System.currentTimeMillis() >= prevTime + 200){
                prevTime = System.currentTimeMillis();
                duckSpinnerPower = -duckSpinnerPower;
            }

            if (duckSpinnersActivated){
                duckSpinner.setPower(duckSpinnerPower);
            }

            if (gamepad1.b && System.currentTimeMillis() >= prevTime + 300) {
                prevTime = System.currentTimeMillis();
                if (duckSpinnersActivated) {
                    duckSpinnersActivated = false;
                    duckSpinner.setPower(0);
                } else {
                    duckSpinnersActivated = true;
                    duckSpinner.setPower(duckSpinnerPower);
                }
                duckSpinner.setHoldPosition(!duckSpinnersActivated);

            }
            if (gamepad1.left_bumper && globalPowerFactor > 0 && System.currentTimeMillis() >= prevTime + 500) {
                prevTime = System.currentTimeMillis();
                globalPowerFactor -= 0.2;
            }
            else if (gamepad1.right_bumper && globalPowerFactor < 1 && System.currentTimeMillis() >= prevTime + 500) {
                prevTime = System.currentTimeMillis();
                globalPowerFactor += 0.2;
            }
            driveTrain.driveRobotCentric(gamepad1.left_stick_y*globalPowerFactor, gamepad1.right_stick_x, gamepad1.left_stick_x*globalPowerFactor);
            if(gamepad2.left_bumper) {
                arm.setTargetPosition(-1750);
            }
            else if (gamepad2.right_bumper){
                arm.setTargetPosition(2500);
                Logger.addData("touchSensor is currently pressed and I am ready to collect!", "#ef3f49");
            }
            else if(gamepad2.y) {
                arm.setTargetPosition(-1000);
            }
            else if(gamepad2.a) {
                arm.setTargetPosition(-370);
            }

            if(armTouchSensor.isPressed() && !lastTouching && lastResetTime+250 < System.currentTimeMillis()){
                arm.resetEncoder();
                arm.setTargetPosition(0);
                arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                Logger.addData("ArmReset: "+ "TRUE");
                lastTouching = true;
                lastResetTime = System.currentTimeMillis();
            }
            else {
                lastTouching = false;
                Logger.addData("ArmReset: "+ "FALSE");
            }

            if(arm.getTargetPosition() >= -2200) {
                if(gamepad2.dpad_down && !armTouchSensor.isPressed()) {
                    arm.setTargetPosition(arm.getCurrentPosition()+125);
                }
                else if(gamepad2.dpad_up) {
                    arm.setTargetPosition(arm.getCurrentPosition()-125);
                }
            }
            arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad2.right_trigger > 0) {
                collector.setPower(gamepad2.right_trigger);
            }
            else if(gamepad2.left_trigger > 0) {
                collector.setPower(-gamepad2.left_trigger);
            }
            else collector.setPower(0);
            
            Logger.addData("GlobalPowerFactor: " + globalPowerFactor);
            Logger.addData("frontRight power: "+ frontRight.getPower());
            Logger.addData("frontLeft power: "+ frontLeft.getPower());
            Logger.addData("backRight power: "+ backRight.getPower());
            Logger.addData("backLeft power: "+ backLeft.getPower());
            Logger.addData("Arm power: "+ arm.getPower());
            Logger.addData("Target arm ticks: "+ arm.getTargetPosition());
            Logger.addData("Current arm ticks: "+ arm.getCurrentPosition());
            Logger.addData("Collector power: "+ collector.getPower());
            Logger.addData("ducksSpinner power: "+ duckSpinner.getPower());
            Logger.addData("ducksSpinner power variable: "+ duckSpinnerPower);
            Logger.addData("frontRight ticks: "+ frontRight.getCurrentPosition());
            Logger.addData("frontLeft ticks: "+ frontLeft.getCurrentPosition());
            Logger.addData("backRight ticks: "+ backRight.getCurrentPosition());
            Logger.addData("backLeft ticks: " + backLeft.getCurrentPosition());
//            Logger.addData("XYZ: "+  imu.getAngles()[0] + " " + imu.getAngles()[1] + " " + imu.getAngles()[2]);
            Logger.update();
        }
    }

}
