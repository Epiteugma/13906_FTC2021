package org.firstinspires.ftc.teamcode.Drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
@TeleOp(name = "FTC 2022 Drive (Mecanum) - OLD", group = "FTC22")
public class OldDriveMecanum extends LinearOpMode {

    @Override
    public void runOpMode() {
        // INIT CODE START HERE

        // Motors and servos
        RevIMU imu = new RevIMU(hardwareMap);
        DcMotor BL = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor BR = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor FL = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor FR = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor duckSpinner1 = hardwareMap.get(DcMotor.class, "duckSpinner1");
        DcMotor duckSpinner2 = hardwareMap.get(DcMotor.class, "duckSpinner2");
        DcMotor claw = hardwareMap.get(DcMotor.class, "arm");
        DcMotor collector = hardwareMap.get(DcMotor.class, "collector");
        Servo capper = hardwareMap.get(Servo.class, "capper");


        // Fix all the directions of the motors.
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU init.
        imu.init();

        // power constants
        double clawPower = 0;
        double duckSpinnerPower = 0;
        double capperPower = 0;
        // timers
        double prevTime = 0;
        // Collector
        boolean isCollectorActive = false;
        boolean collectorDirection = false;
        // power factors
        double multiplier = 0.65;
        double globalpowerfactor = 1.0;
        //END INIT CODE

        // wait for user to press start
        waitForStart();

        // AFTER START CODE HERE

        while (opModeIsActive()) {
//            Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double forwardpower = gamepad1.left_stick_y * globalpowerfactor;
            double sidepower = gamepad1.left_stick_x * globalpowerfactor;
            double turnpower = gamepad1.right_stick_x * globalpowerfactor;

            if(gamepad1.right_bumper && System.currentTimeMillis() > prevTime+200) {
                prevTime = System.currentTimeMillis();
                globalpowerfactor = 0.8;
            }
            else if(gamepad1.left_bumper && System.currentTimeMillis() > prevTime+200) {
                prevTime = System.currentTimeMillis();
                globalpowerfactor = 0.3;
            }

            // Smooth out the power. Max is 1 if power is more than 1 it will confuse itself and like this we keep the ratio, the same without wasting power.
            double denominator = Math.max(Math.abs(forwardpower), Math.max(Math.abs(sidepower), Math.abs(turnpower)));

            // Calculate DC Motor Powers
            double frPower = (forwardpower - sidepower - turnpower) / denominator;
            double flPower = (forwardpower + sidepower + turnpower) / denominator;
            double brPower = (forwardpower + sidepower - turnpower) / denominator;
            double blPower = (forwardpower - sidepower + turnpower) / denominator;

            FR.setPower(frPower);
            FL.setPower(flPower);
            BR.setPower(brPower);
            BL.setPower(blPower);


            // CAPPER AND ARMCLAW CODE
            if(gamepad2.left_trigger == 0 && gamepad2.dpad_up && System.currentTimeMillis() > prevTime+200) {
                prevTime = System.currentTimeMillis();
                clawPower = multiplier;
            }
            else if(gamepad2.left_trigger == 0 && gamepad2.dpad_down && System.currentTimeMillis() > prevTime+200) {
                prevTime = System.currentTimeMillis();
                clawPower = -multiplier;
            }
            else if(gamepad2.left_trigger != 0 && gamepad2.dpad_up) {
                capperPower = 1;
            }
            else if(gamepad2.left_trigger != 0 && gamepad2.dpad_down) {
                capperPower = -1;
            }
            else if(gamepad2.dpad_left && System.currentTimeMillis() > prevTime+200) {
                prevTime = System.currentTimeMillis();
                clawPower = 0;
            }
            else {
                capperPower = 0;
            }
            //capper.setPower(capperPower);
            claw.setPower(clawPower);


            // INTAKE CODE
            if(gamepad2.cross && System.currentTimeMillis() > prevTime+200) {
                prevTime = System.currentTimeMillis();
                if(isCollectorActive && collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = true;
                }
            } else if(gamepad2.triangle && System.currentTimeMillis() > prevTime+200) {
                prevTime = System.currentTimeMillis();
                if(isCollectorActive && !collectorDirection) { isCollectorActive = false; }
                else {
                    isCollectorActive = true;
                    collectorDirection = false;
                }
            }

            // DUCK SPINNER CODE
            if(gamepad1.square && System.currentTimeMillis() > prevTime+200) {
                prevTime = System.currentTimeMillis();
                duckSpinnerPower = duckSpinnerPower == multiplier-0.25 ? 0 : multiplier-0.25;
            }
            duckSpinner1.setPower(duckSpinnerPower);
            duckSpinner2.setPower(duckSpinnerPower);

            if(isCollectorActive) {
                collector.setPower(collectorDirection ? 1 : -multiplier);
            } else collector.setPower(0);

            // Telemetry
            telemetry.addData("GlobalPowerFactor: ", globalpowerfactor);
            // telemetry.addData("Turn amount: ", calculateAngle360(angles.firstAngle));
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