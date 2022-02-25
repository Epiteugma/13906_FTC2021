package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Autonomous(name="Red Alliance Right", group="FTC22")
public class RedAllianceRight extends LinearOpMode {

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
    public void runOpMode(){
        initHardware();
        Robot robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, arm, collector, duckSpinners, imu, cargoDetector), this);

//        Detector detector = new Detector(hardwareMap);
//        Detector.ElementPosition itemPos = detector.getElementPosition();
//        telemetry.addData("Detected Cargo : ", robot.cargoDetection());
//        telemetry.update();
        waitForStart();
//        switch (itemPos) {
//            case LEFT:
//                robot.drive(Robot.Direction.LEFT, 0.5, 5000);
//                break;
//            case RIGHT:
//                robot.strafe(Robot.Direction.LEFT, 0.5, 5000);
//                break;
//            case CENTER:
        robot.drive(Robot.Direction.BACKWARDS, 0.3, 100);
        //break;
//        }
    }
}
