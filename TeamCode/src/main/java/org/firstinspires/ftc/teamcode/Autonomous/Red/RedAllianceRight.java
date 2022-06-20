package org.firstinspires.ftc.teamcode.Autonomous.Red;

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

@Autonomous(name="Red Right", group="FTC22Auto_Ware_Red")
public class RedAllianceRight extends LinearOpMode {

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

    double secondsRemaining = 30;
    double opModeStartTime = System.currentTimeMillis();

    private void initHardware() {
        // Motors, servos, distance sensor and IMU
        imu = new RevIMU(hardwareMap);
        cargoDetector = hardwareMap.get(DistanceSensor.class, "cargoDetector");

        duckSpinner = new Motor(hardwareMap, "duckSpinner");
        arm = new Motor(hardwareMap, "arm");
        collector = new Motor(hardwareMap, "collector");
        capper= new CRServo(hardwareMap, "capper");
        frontRight = new Motor(hardwareMap, "frontRight");
        frontLeft = new Motor(hardwareMap, "frontLeft");
        backRight = new Motor(hardwareMap, "backRight");
        backLeft = new Motor(hardwareMap, "backLeft");
    }

    @Override
    public void runOpMode() {
        initHardware();
        Robot robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, arm, collector, duckSpinner, imu, cargoDetector,frontDistance), this);

        waitForStart();
        TseDetector.Location itemPos = robot.getTsePos();
        telemetry.addData("Detected Cargo: ", itemPos);
        telemetry.update();
        new Thread(() -> {
            while(opModeIsActive()) {
                double secondsSinceStart = Math.floor((System.currentTimeMillis() - opModeStartTime)/1000);
                secondsRemaining = 30 - secondsSinceStart;
            }
        }).start();
        robot.drive(Robot.Direction.FORWARDS, 0.8, 10);
        robot.turn(0.8, 90);
        robot.drive(Robot.Direction.FORWARDS, 0.8, 57);
        robot.turn(0.8, 0);
        switch (itemPos) {
            case LEFT: robot.moveArm(Robot.Position.LOW.label, 0.5); break;
            case RIGHT: robot.moveArm(Robot.Position.HIGH.label, 0.5); break;
            case CENTER: robot.moveArm(Robot.Position.MID.label, 0.5); break;
        }
        robot.drive(Robot.Direction.BACKWARDS, 0.08, 0.01); // UNKNOWN BUG!!!
        robot.drive(Robot.Direction.FORWARDS, 0.8, 35);
        robot.turn(1, 0);
        switch (itemPos) {
            case LEFT: robot.intake(Robot.Direction.OUT, 0.5); break;
            case RIGHT: switch (itemPos) {
            case LEFT: robot.intake(Robot.Direction.OUT, robot.intakeLowSpeed); break;
            case RIGHT: robot.intake(Robot.Direction.OUT, robot.intakeHighSpeed); break;
            case CENTER: robot.intake(Robot.Direction.OUT, robot.intakeMidSpeed); break;
        } break;
            case CENTER: switch (itemPos) {
            case LEFT: robot.intake(Robot.Direction.OUT, robot.intakeLowSpeed); break;
            case RIGHT: robot.intake(Robot.Direction.OUT, robot.intakeHighSpeed); break;
            case CENTER: robot.intake(Robot.Direction.OUT, robot.intakeMidSpeed); break;
        } break;
        }
        robot.turn(1, 0);
        robot.drive(Robot.Direction.BACKWARDS, 1, 25);
        robot.turn(1, -90);
        robot.moveArm(Robot.Position.MID.label, 0.08);
        robot.drive(Robot.Direction.FORWARDS, 1, 160);
    }
}