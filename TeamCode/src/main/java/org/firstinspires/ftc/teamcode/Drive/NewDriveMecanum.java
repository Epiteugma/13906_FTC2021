package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.z3db0y.susanalib.MecanumDriveTrain;
import com.z3db0y.susanalib.Motor;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public void addData(String tag, String data, String tagCol, String dataCol) {
        if(dataCol == null) dataCol = "#e37b29";
        if(tagCol == null) tagCol = "#e37b29";
        tag = tag.replaceAll("<","&lt").replaceAll(">","&gt;").replaceAll(" ","&nbsp;").replaceAll("\"","&quot").replaceAll("'","&apos;");
        data = data.replaceAll("<","&lt").replaceAll(">","&gt;").replaceAll(" ","&nbsp;").replaceAll("\"","&quot").replaceAll("'","&apos;");
        telemetry.addData(String.format("<span style='color: %s'>%s</span>", tagCol, tag), String.format("<span style='color: %s'>%s</span>", dataCol, data));
    }

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Colored telemetry (basically html)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        double duckSpinnerPower = 0.25;
        double globalPowerFactor = 0.65;
        boolean duckSpinnersActivated = false;
        long prevTime = 0;
        initMotors();
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        double prevDistance = distance;
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(frontLeft, frontRight, backLeft, backRight);
        waitForStart();
        while(opModeIsActive()) {
            distance = distanceSensor.getDistance(DistanceUnit.CM);
            addData("PrevDistance", String.valueOf(prevDistance), null, null);
            addData("Distance: ", String.valueOf(distance), null, null);
            if(Math.abs(distance - prevDistance) >= 1) {
//                gamepad1.rumble(1000);
//                gamepad2.rumble(1000);
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
            if(gamepad2.y) {
                arm.setTargetPosition(-1700);
            } else if(gamepad2.b) {
                arm.setTargetPosition(-1000);
            } else if(gamepad2.a) {
                arm.setTargetPosition(-370);
            }

            if(!armTouchSensor.isPressed() || arm.getTargetPosition() == 0) {
                if(gamepad2.dpad_down) {
                    arm.setTargetPosition(arm.getCurrentPosition()+75);
                } else if(gamepad2.dpad_up) {
                    arm.setTargetPosition(arm.getCurrentPosition()-75);
                }
            } else {
                addData("ArmReset: ", "true", null, null);
                arm.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(0);
            }
            arm.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad2.right_trigger > 0) {
                collector.setPower(gamepad2.right_trigger);
            } else if(gamepad2.left_trigger > 0) {
                collector.setPower(-gamepad2.left_trigger);
            } else collector.setPower(0);

            addData("Target arm ticks", String.valueOf(arm.getTargetPosition()), null, null);
            addData("Current arm ticks", String.valueOf(arm.getCurrentPosition()), null, null);
            telemetry.update();
        }
    }

}
