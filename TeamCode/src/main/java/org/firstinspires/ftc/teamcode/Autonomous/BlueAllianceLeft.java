package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

@Autonomous(name="Blue Alliance Left", group="FTC22")
public class BlueAllianceLeft extends LinearOpMode {

    Motor arm;
    Motor collector;
    CRServo capper;
    Motor frontRight;
    Motor frontLeft;
    Motor backRight;
    Motor backLeft;
    MotorGroup duckSpinners;

    private void initMotors() {
        // Motors, servos, distance sensor and IMU
        BNO055IMU IMU = hardwareMap.get(BNO055IMU.class, "imu");
        SensorRevTOFDistance cargoDetector = new SensorRevTOFDistance(hardwareMap, "cargoDetector");
        Motor duckSpinner1 = new Motor( hardwareMap, "duckSpinner1");
        Motor duckSpinner2 = new Motor( hardwareMap, "duckSpinner2");
        duckSpinners = new MotorGroup(duckSpinner1, duckSpinner2);
        arm = new Motor(hardwareMap, "arm");
        collector = new Motor(hardwareMap, "collector");
        CRServo capper= new CRServo(hardwareMap, "capper");
        frontRight = new Motor(hardwareMap, "frontRight");
        frontLeft = new Motor(hardwareMap, "frontLeft");
        backRight = new Motor(hardwareMap, "backRight");
        backLeft = new Motor(hardwareMap, "backLeft");
    }

    @Override
    public void runOpMode(){
        initMotors();
        Robot robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, arm, collector, duckSpinners), this);

//        Detector detector = new Detector(hardwareMap);
//        Detector.ElementPosition itemPos = detector.getElementPosition();
//        telemetry.addData("Detected Cargo : ", robot.cargoDetection());
//        telemetry.update();
//        waitForStart();
//        switch (itemPos) {
//            case LEFT:
//                robot.drive(Robot.Direction.LEFT, 0.5, 5000);
//                break;
//            case RIGHT:
//                robot.strafe(Robot.Direction.LEFT, 0.5, 5000);
//                break;
//            case CENTER:
        robot.drive(Robot.Direction.FORWARDS, 0.5, 15);
                //break;
//        }
    }
}
