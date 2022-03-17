package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Robot;
import org.firstinspires.ftc.teamcode.Autonomous.visionv1.TseDetector;

import java.util.Arrays;

@Autonomous(name="Red Alliance Right", group="FTC22Auto_Ware_Red")
public class RedAllianceRight extends LinearOpMode {

    Motor arm;
    Motor collector;
    CRServo capper;
    Motor frontRight;
    Motor frontLeft;
    Motor backRight;
    Motor backLeft;
    Motor duckSpinner1;
    Motor duckSpinner2;
    MotorGroup duckSpinners;
    RevIMU imu;
    SensorColor cargoDetector;
    SensorRevTOFDistance frontDistance;

    double secondsRemaining = 30;
    double opModeStartTime = System.currentTimeMillis();

    private void initHardware() {
        // Motors, servos, distance sensor and IMU
        imu = new RevIMU(hardwareMap);
        cargoDetector = new SensorColor(hardwareMap,"cargoDetector");
        frontDistance = new SensorRevTOFDistance(hardwareMap,"frontDistance");
        duckSpinner1 = new Motor(hardwareMap, "duckSpinner1"); // Left
        duckSpinner2 = new Motor(hardwareMap, "duckSpinner2"); // Right
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
        Robot robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, arm, collector, duckSpinner2, imu, cargoDetector,frontDistance), this);

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
        robot.drive(Robot.Direction.FORWARDS, 0.8, 58);
        robot.turn(0.8, 0);
        switch (itemPos) {
            case LEFT: robot.moveArm(Robot.Position.LOW.label, 0.5); break;
            case RIGHT: robot.moveArm(Robot.Position.HIGH.label, 0.5); break;
            case CENTER: robot.moveArm(Robot.Position.MID.label, 0.5); break;
        }
        robot.drive(Robot.Direction.BACKWARDS, 0.08, 0.01); // UNKNOWN BUG!!!
        robot.drive(Robot.Direction.FORWARDS, 0.8, 38);
        robot.intake(Robot.Direction.OUT, 0.65);
        robot.turn(1, 0);
        robot.drive(Robot.Direction.BACKWARDS, 1, 43);
        robot.turn(1, -90);
        robot.moveArm(Robot.Position.MID.label, 0.08);
        robot.drive(Robot.Direction.FORWARDS, 1, 140);
//            robot.pause(3000);
//            robot.driverOverBarriers(Robot.Direction.FORWARDS,1);
////            robot.pause(3000);
//            robot.moveArm(Robot.Position.DOWN.label, 0.08);
//            while(robot.cargoDetection().equals("None") && opModeIsActive()) {
//                if(robot.overCargo()) {
//                    robot.drive(Robot.Direction.BACKWARDS, 1, 25);
//                    robot.turn(1,45);
//                    robot.pause(3000);
//                }
//                double speed = 0.3;
//                collector.set(1);
//                robot.setAllDrivePower(speed);
//            }
//            robot.HALT();
//
////            if(secondsRemaining < 7) break;
//
//            telemetry.addData("Cube counter:", cubeCounter);
//            telemetry.addData("Cube score counter:", cubeCounter*6);
//            telemetry.addData("Seconds remaining:", secondsRemaining);
//            telemetry.addData("Seconds running:", 30-secondsRemaining);
//            telemetry.addData("Just Collected: ", robot.cargoDetection());
//            telemetry.update();
//
//            robot.moveArm(Robot.Position.MID.label, 0.08);
//            robot.turn(1, -90);
//            robot.driverOverBarriers(Robot.Direction.BACKWARDS,1);
//            robot.drive(Robot.Direction.BACKWARDS, 1, 120);
//            robot.turn(1, 0);
//            robot.moveArm(Robot.Position.HIGH.label, 0.08);
//            robot.drive(Robot.Direction.FORWARDS, 1, 60);
//            robot.intake(Robot.Direction.OUT, 0.65);
//
//            cubeCounter++;
//        }
    }
}