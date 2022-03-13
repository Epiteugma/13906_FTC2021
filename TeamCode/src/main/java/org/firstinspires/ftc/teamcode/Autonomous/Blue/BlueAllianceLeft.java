package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.ejml.equation.IntegerSequence;
import org.firstinspires.ftc.teamcode.Autonomous.Robot;
import org.firstinspires.ftc.teamcode.Autonomous.visionv1.TseDetector;

import java.util.Arrays;

@Autonomous(name="Blue Alliance Left", group="FTC22Auto_Ware_Blue")
public class BlueAllianceLeft extends LinearOpMode {

    Motor arm;
    Motor collector;
    CRServo capper;
    Motor frontRight;
    Motor frontLeft;
    Motor backRight;
    Motor backLeft;
    MotorGroup duckSpinners;
    RevIMU imu;
    SensorRevTOFDistance cargoDetector;

    int cubeCounter = 0;
    double secondsRemaining = 30;
    double opModeStartTime = System.currentTimeMillis();

    private void initHardware() {
        // Motors, servos, distance sensor and IMU
        imu = new RevIMU(hardwareMap);
        cargoDetector = new SensorRevTOFDistance(hardwareMap, "cargoDetector");
        Motor duckSpinner1 = new Motor( hardwareMap, "duckSpinner1");
        Motor duckSpinner2 = new Motor( hardwareMap, "duckSpinner2");
        duckSpinners = new MotorGroup(duckSpinner1, duckSpinner2);
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
        Robot robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, arm, collector, duckSpinners, imu, cargoDetector), this);

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
        robot.drive(Robot.Direction.FORWARDS, 0.8, 20);
        if (itemPos.equals(TseDetector.Location.LEFT)) {
            robot.turn(1, -40);
            robot.drive(Robot.Direction.FORWARDS, 0.8, 30);
            robot.moveArm(Robot.Position.LOW.label, 0.08);
        } else if (itemPos.equals(TseDetector.Location.RIGHT)) {
            robot.turn(1, -75);
            robot.drive(Robot.Direction.FORWARDS, 0.8, 70);
            robot.turn(1, 0);
            robot.moveArm(Robot.Position.HIGH.label, 0.08);
        }
        // CENTER
        else {
            robot.turn(1, -65);
            robot.drive(Robot.Direction.FORWARDS, 0.8, 35);
            robot.turn(1, -20);
            robot.moveArm(Robot.Position.MID.label, 0.08);
        }
        robot.intake(Robot.Direction.OUT, 0.45);
        while (opModeIsActive()) {
            robot.turn(1, 0);
            robot.drive(Robot.Direction.BACKWARDS, 1, 20);
            robot.turn(1, 90);
            robot.moveArm(Robot.Position.MID.label, 0.08);
            robot.drive(Robot.Direction.FORWARDS, 1, 80);
            robot.moveArm(Robot.Position.DOWN.label, 0.08);
            double startTime = System.currentTimeMillis();
            while(robot.cargoDetection().equals("None") && opModeIsActive()) {
                if(System.currentTimeMillis() > startTime+3000) {
                    startTime = System.currentTimeMillis();
                    robot.drive(Robot.Direction.BACKWARDS, 1, 25);
                }
                double speed = 0.4;
                collector.set(1);
                frontLeft.set(speed);
                frontRight.set(speed);
                backLeft.set(speed);
                backRight.set(speed);
            }
            frontLeft.set(0);
            frontRight.set(0);
            backLeft.set(0);
            backRight.set(0);
            collector.set(0);
            telemetry.addData("Cube counter:", cubeCounter);
            telemetry.addData("Cube score counter:", cubeCounter*6);
            telemetry.addData("Seconds remaining:", secondsRemaining);
            telemetry.addData("Seconds running:", 30-secondsRemaining);
            telemetry.addData("Collected: ", robot.cargoDetection());
            telemetry.update();

            if(secondsRemaining < 7) break;

            robot.turn(1, 90);
            robot.moveArm(Robot.Position.HIGH.label, 0.08);
            robot.drive(Robot.Direction.BACKWARDS, 1, 105);
            robot.turn(1, -30);
            robot.drive(Robot.Direction.FORWARDS, 1, 40);
            robot.moveArm(Robot.Position.HIGH.label, 0.08);
            robot.intake(Robot.Direction.OUT, 0.45);

            cubeCounter++;
        }
    }
}