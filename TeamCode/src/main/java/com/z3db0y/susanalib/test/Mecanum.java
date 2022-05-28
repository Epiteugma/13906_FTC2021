package com.z3db0y.susanalib.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

//@Disabled
@TeleOp(name = "Mecanum Robot Centric Test", group = "SusanaLib")
public class Mecanum extends LinearOpMode {

    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;

    private void initMotors() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");
        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {
        initMotors();

        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight);

        waitForStart();
        while (opModeIsActive()) {
            driveTrain.driveRobotCentric(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
        }
    }

}
