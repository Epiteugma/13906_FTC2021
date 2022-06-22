package org.firstinspires.ftc.teamcode.Autonomous.Old.Blue;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Robot;
import org.firstinspires.ftc.teamcode.Autonomous.visionv1.TseDetector;

import java.util.Arrays;

@Autonomous(name = "Blue Right Storage Unit", group="FTC22Auto_Store")
public class BlueAllianceRightStorageUnit extends LinearOpMode {

    Motor arm;
    Motor collector;
    CRServo capper;
    Motor frontRight;
    Motor frontLeft;
    Motor backRight;
    Motor backLeft;
    Motor duckSpinner;
    
    RevIMU imu;
    DistanceSensor cargoDetector;
    SensorRevTOFDistance frontDistance;

    public void initHardware() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");
        arm = new Motor(hardwareMap, "arm");
        collector = new Motor(hardwareMap, "collector");
        duckSpinner = new Motor(hardwareMap, "duckSpinner");

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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    @Override
    public void runOpMode() {
        initHardware();
        Robot robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, arm, collector, duckSpinner, imu, cargoDetector, frontDistance), this);

        waitForStart();
        TseDetector.Location itemPos = robot.getTsePos();
        telemetry.addData("Detected Cargo: ", itemPos);
        telemetry.update();
        robot.drive(Robot.Direction.FORWARDS, 0.8, 10);
        robot.turn(0.8, 90);
        robot.drive(Robot.Direction.FORWARDS, 0.8, 55);
        robot.turn(0.8, 0);
        robot.drive(Robot.Direction.BACKWARDS, 0.8, 0.01); // UNKNOWN BUG!!!
        switch (itemPos) {
            case LEFT:
                robot.moveArm(Robot.Position.LOW.label, 0.5);
                break;
            case RIGHT:
                robot.moveArm(Robot.Position.HIGH.label, 0.5);
                break;
            case CENTER:
                robot.moveArm(Robot.Position.MID.label, 0.5);
                break;
        }
        robot.drive(Robot.Direction.FORWARDS, 0.8, 33);
        robot.turn(0.5, 0);
        switch (itemPos) {
            case LEFT: robot.intake(Robot.Direction.OUT, robot.disposeLowSpeed); break;
            case RIGHT: robot.intake(Robot.Direction.OUT, robot.disposeHighSpeed); break;
            case CENTER: robot.intake(Robot.Direction.OUT, robot.disposeMidSpeed); break;
        }
        robot.drive(Robot.Direction.BACKWARDS, 0.4, 25);
        robot.moveArm(Robot.Position.DOWN.label, 0.08);
        robot.turn(0.8, -93);
        robot.drive(Robot.Direction.FORWARDS, 0.8, 127);
        robot.turn(0.8, -92);
        robot.duckSpin(-0.275, 4000);
        robot.turn(1, 0);
        robot.drive(Robot.Direction.BACKWARDS, 0.8, 0.01); // UNKNOWN BUG!!!
        robot.drive(Robot.Direction.FORWARDS, 0.45, 50);
        robot.turn(1, 0);
    }
}