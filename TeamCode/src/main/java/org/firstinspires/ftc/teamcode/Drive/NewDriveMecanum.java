package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

@TeleOp(name = "Beta drive", group = "FTC22")
public class NewDriveMecanum extends LinearOpMode {
    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    Motor arm;
    Motor collector;

    public void initMotors() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");
        arm = new Motor(hardwareMap, "arm");
        collector = new Motor(hardwareMap, "collector");

        backLeft.setDirection(Motor.Direction.REVERSE);
        frontRight.setDirection(Motor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setTargetPosition(0);
        arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    @Override
    public void runOpMode() {
        double globalPowerFactor = 1;
        long prevTime = 0;
        initMotors();
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.left_bumper && globalPowerFactor > 0 && System.currentTimeMillis() >= prevTime + 500) {
                prevTime = System.currentTimeMillis();
                globalPowerFactor -= 0.2;
            } else if (gamepad1.right_bumper && globalPowerFactor < 1 && System.currentTimeMillis() >= prevTime + 500) {
                prevTime = System.currentTimeMillis();
                globalPowerFactor += 0.2;
            }
            driveTrain.driveRobotCentric(gamepad1.left_stick_y*globalPowerFactor, gamepad1.right_stick_x*globalPowerFactor, gamepad1.left_stick_x*globalPowerFactor);
            if(gamepad2.y) {
                arm.setTargetPosition(-1700);
            } else if(gamepad2.b) {
                arm.setTargetPosition(-1000);
            } else if(gamepad2.a) {
                arm.setTargetPosition(-370);
            }

            if(gamepad2.dpad_down) {
                arm.setTargetPosition(arm.getCurrentPosition()+75);
            } else if(gamepad2.dpad_up) {
                arm.setTargetPosition(arm.getCurrentPosition()-75);
            }
            arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad2.right_trigger > 0) {
                collector.setPower(gamepad2.right_trigger);
            } else if(gamepad2.left_trigger > 0) {
                collector.setPower(-gamepad2.left_trigger);
            } else collector.setPower(0);
        }
    }

}
