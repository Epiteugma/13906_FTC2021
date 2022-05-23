package org.firstinspires.ftc.teamcode.SusanaMini;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic Mecanum Drive", group = "FTC22")
public class Basic extends LinearOpMode {
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
//    DcMotor platform;

    public void turnPlatform(boolean dpad_left, boolean dpad_right) {
//        if(dpad_left && !dpad_right) platform.setPower(-1);
//        else if(dpad_right && !dpad_left) platform.setPower(1);
//        else platform.setPower(0);
    }

    public void initMotors() {
        front_left = hardwareMap.get(DcMotor.class, "frontLeft");
        front_right = hardwareMap.get(DcMotor.class, "frontRight");
        back_left = hardwareMap.get(DcMotor.class, "backLeft");
        back_right = hardwareMap.get(DcMotor.class, "backRight");
//        platform = hardwareMap.get(DcMotor.class, "platform");

        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        platform.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void drive(double forward_power, double strafe_power, double side_power) {
        front_left.setPower(forward_power - side_power - strafe_power);
        front_right.setPower(forward_power + side_power + strafe_power);
        back_left.setPower(forward_power - side_power + strafe_power);
        back_right.setPower(forward_power + side_power - strafe_power);

//        front_left.setPower(forward_power);
//        front_right.setPower(forward_power);
//        back_left.setPower(forward_power);
//        back_right.setPower(forward_power);
    }

    @Override
    public void runOpMode() {
        initMotors();
        waitForStart();

        while (opModeIsActive()) {
            drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

}
