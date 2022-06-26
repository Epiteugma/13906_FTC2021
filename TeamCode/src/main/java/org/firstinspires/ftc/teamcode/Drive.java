package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.z3db0y.susanalib.Logger;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Teleop", group = "FTC22")
public class Drive extends LinearOpMode {
    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    Motor arm;
    Motor collector;
    Motor duckSpinner;
    DistanceSensor cargoDetector;
    DistanceSensor backDistance;
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");
    }

    double duckSpinnerPower = 0.25;
    double globalPowerFactor = 0.65;
    boolean lastTouching = false;
    boolean duckSpinnersActivated = false;
    double lastResetTime = 0;
    long prevTime = 0;

    private void duckSpinnerControl(){
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
    }

    private void armControl(){
        if(gamepad2.left_bumper) {
            arm.setTargetPosition(-1750);
        }

        else if (gamepad2.right_bumper){
            arm.setTargetPosition(2500);
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
            lastTouching = true;
            Logger.addData("touchSensor is currently pressed and I am ready to collect!", "#ef3f49");
            lastResetTime = System.currentTimeMillis();
        }
        else {
            lastTouching = false;
            Logger.addData("ArmReset: "+ "False");
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
    }

    private void globalPowerFactorControl(){
        if (gamepad1.left_bumper && globalPowerFactor > 0 && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor -= 0.2;
        }
        else if (gamepad1.right_bumper && globalPowerFactor < 1 && System.currentTimeMillis() >= prevTime + 500) {
            prevTime = System.currentTimeMillis();
            globalPowerFactor += 0.2;
        }
    }

    private void collectorControl(){
        if(gamepad2.right_trigger > 0) {
            collector.setPower(gamepad2.right_trigger);
        }
        else if(gamepad2.left_trigger > 0) {
            collector.setPower(-gamepad2.left_trigger);
        }
        else collector.setPower(0);
    }

    private void Logging(){
        Logger.addData("Powers:");
        Logger.addData("    GlobalPowerFactor: " + globalPowerFactor);
        Logger.addData("    frontRight power: "+ frontRight.getPower());
        Logger.addData("    frontLeft power: "+ frontLeft.getPower());
        Logger.addData("    backRight power: "+ backRight.getPower());
        Logger.addData("    backLeft power: "+ backLeft.getPower());
        Logger.addData("    Arm power: "+ arm.getPower());
        Logger.addData("    Collector power: "+ collector.getPower());
        Logger.addData("Ticks:");
        Logger.addData("    Current arm ticks: "+ arm.getCurrentPosition());
        Logger.addData("    Target arm ticks: "+ arm.getTargetPosition());
        Logger.addData("    ducksSpinner power: "+ duckSpinner.getPower());
        Logger.addData("    ducksSpinner power variable: "+ duckSpinnerPower);
        Logger.addData("    frontRight ticks: "+ frontRight.getCurrentPosition());
        Logger.addData("    frontLeft ticks: "+ frontLeft.getCurrentPosition());
        Logger.addData("    backRight ticks: "+ backRight.getCurrentPosition());
        Logger.addData("    backLeft ticks: " + backLeft.getCurrentPosition());
        Logger.update();
    }

    @Override
    public void runOpMode() {

        initHardware();
        Logger.setTelemetry(telemetry);

        double distance = cargoDetector.getDistance(DistanceUnit.CM);
        double prevDistance = distance;

        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight, imu);
        waitForStart();
        while(opModeIsActive()) {
            distance = cargoDetector.getDistance(DistanceUnit.CM);
            Logger.addData("PrevDistance: " + prevDistance);
            Logger.addData("Distance: " + distance);
            
            if(Math.abs(prevDistance - distance) >= 2) {
                gamepad1.rumble(1, 0.5, 100);
                gamepad2.rumble(1, 0.5, 100);
                prevDistance = distance;
            }

            if(driveTrain.isStalled()) {
                Logger.addData("Drivetrain is stalled");
                gamepad1.rumble(0.5, 1, 1000);
                gamepad2.rumble(0.5, 1, 1000);
            }

            duckSpinnerControl();

            globalPowerFactorControl();

            driveTrain.driveRobotCentric(gamepad1.left_stick_y*globalPowerFactor, gamepad1.right_stick_x, gamepad1.left_stick_x*globalPowerFactor);
            
            armControl();

            collectorControl();

            Logging();
        }
    }

}
