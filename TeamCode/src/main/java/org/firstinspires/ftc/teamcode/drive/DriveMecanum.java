package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "FTC 2022 Drive (Mecanum)", group = "FTC22")
public class DriveMecanum extends LinearOpMode {

    @Override
    public void runOpMode() {
        // INIT CODE START HERE

        // Motors and servos
        BNO055IMU IMU = hardwareMap.get(BNO055IMU.class, "imu");
        DcMotor BL = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor BR = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor FL = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor FR = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor duckSpinner1 = hardwareMap.get(DcMotor.class, "duckSpinner1");
        DcMotor duckSpinner2 = hardwareMap.get(DcMotor.class, "duckSpinner2");
        MotorGroup duckSpinners = new MotorGroup(duckSpinner1, duckSpinner2);
        DcMotor claw = hardwareMap.get(DcMotor.class, "armClaw");
        DcMotor collector = hardwareMap.get(DcMotor.class, "collector");
        CRServo capper = hardwareMap.get(CRServo.class, "capper");


        // Fix all the directions of the motors.
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU init.
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled = true;
        params.loggingTag = "IMU";
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        // power constants
        double clawPower = 0;
        double duckSpinnersPower = 0;
        double capperPower = 0;
        // timers
        double prevTime = 0;
        // Collector
        boolean isCollectorActive = false;
        boolean collectorDirection = false;
        // power factors
        double multiplier = 0.6;
        double globalpowerfactor = 1.0;
        // Gamepads init
        GamepadEx gamepad1 = new Gamepad(gamepad1);
        GamepadEx gamepad2 = new Gamepad(gamepad2);
        // Meccanum drivebase
        MecanumDrive meccanumDrive = new MecanumDrive(FL, FR, BL, BR);

        //END INIT CODE

        // wait for user to press start
        waitForStart();

        // AFTER START CODE HERE

        while (opModeIsActive()) {
            Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double forwardpower = gamepad1.getLeftY() * globalpowerfactor;
            double sidepower = gamepad1.getLeftX() * globalpowerfactor;
            double turnpower = gamepad1.getRightX() * globalpowerfactor;

            if(gamepad1.wasJustPressed(GamepadKeys.RIGHT_BUMPER)) {
                globalpowerfactor = 0.8;
            }
            else if(gamepad1.wasJustPressed(GamepadKeys.LEFT_BUMPER)) {
                globalpowerfactor = 0.3;
            }

            // Smooth out the power. Max is 1 if power is more than 1 it will confuse itself and like this we keep the ratio, the same without wasting power.
            double denominator = Math.max(Math.abs(forwardpower), Math.max(Math.abs(sidepower), Math.abs(turnpower)));

            // Calculate DC Motor Powers
            // frPower = (forwardpower - sidepower - turnpower) / denominator;
            // flPower = (forwardpower + sidepower + turnpower) / denominator;
            // brPower = (forwardpower + sidepower - turnpower) / denominator;
            // blPower = (forwardpower - sidepower + turnpower) / denominator;

            mecanumDrive.driveRobotCentric(sidepower, forwardpower, turnpower);
            FR.setPower(frPower);
            FL.setPower(flPower);
            BR.setPower(brPower);
            BL.setPower(brPower);


            // ArmClaw up DPAD_UP
            if(gamepad2.wasJustPressed(GamepadKeys.LEFT_TRIGGER) != true && gamepad2.wasJustPressed(GamepadKeys.DPAD_UP)) {
                clawPower = multiplier;
            }
            // ArmClaw down DPAD_DOWN
            else if(gamepad2.wasJustPressed(GamepadKeys.LEFT_TRIGGER) != true && gamepad2.wasJustPressed(GamepadKeys.DPAD_DOWN)) {
                clawPower = -multiplier;
            }
            // Capper up LEFT_TRIGGER AND DPAD_UP
            else if(gamepad2.wasJustPressed(GamepadKeys.LEFT_TRIGGER) && gamepad2.wasJustPressed(GamepadKeys.DPAD_UP)) {
                capperPower = 1;
            }
            // Capper down LEFT_TRIGGER AND DPAD_DOWN
            else if(gamepad2.wasJustPressed(GamepadKeys.LEFT_TRIGGER) && gamepad2.wasJustPressed(GamepadKeys.DPAD_DOWN)) {
                capperPower = -1;
            }
            // Stop armClaw DPAD_LEFT
            else if(gamepad2.wasJustPressed(GamepadKeys.DPAD_LEFT)) {
                clawPower = 0;
            }
            // Stop Capper
            else {
                capperPower = 0;
            }
            capper.setPower(capperPower);
            claw.setPower(clawPower);


            // INTAKE CODE
          
            if(gamepad2.wasJustPressed(GamepadKeys.cross)) {
                if(isCollectorActive && collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = true;
                }
            } else if(gamepad2.wasJustPressed(GamepadKeys.triangle)) {
                if(isCollectorActive && !collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = false;
                }
            }

            // DUCK SPINNER CODE
            if(gamepad1.wasJustPressed(GamepadKeys.square)) {
                duckSpinnersPower = duckSpinnersPower == multiplier-0.25 ? 0 : multiplier-0.25;
            }
            duckSpinners.setPower(duckSpinnersPower);

            if(isCollectorActive) {
                collector.setPower(collectorDirection ? 1 : -multiplier);
            } else collector.setPower(0);

            // Telemetry
            telemetry.addData("GlobalPowerFactor: ", globalpowerfactor);
            telemetry.addData("Turn amount: ", calculateAngle360(angles.firstAngle));
            telemetry.addData("FL: ", (forwardpower - sidepower + turnpower));
            telemetry.addData("BL: ", (forwardpower + sidepower + turnpower));
            telemetry.addData("FR: ", -(forwardpower + sidepower - turnpower));
            telemetry.addData("BR: ", -(forwardpower - sidepower - turnpower));
            telemetry.addData("Claw :", clawPower);
            telemetry.addData("Collector: ", collector.getPower());
            telemetry.update();
        }
    }

    private double makePositive(double num) {
        if(num < 0) return -num;
        return num;
    }

    private double calculateAngle360(double num) {
        if(num < 0) return 360+num;
        return num;
    }
}
