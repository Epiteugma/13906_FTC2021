package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.visionv1.TseDetector;

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
        TseDetector.Location itemPos = robot.getTsePos();
        telemetry.addData("Detected Cargo: ", itemPos);
        telemetry.update();
        waitForStart();
        robot.turn(Robot.Direction.RIGHT, 0.3, 90);
        robot.drive(Robot.Direction.FORWARDS,0.3,100);
        robot.turn(Robot.Direction.LEFT, 0.3, 90);
        switch (itemPos) {
            case LEFT:
                robot.moveArm(Robot.Position.LOW,0.1);
                break;
            case CENTER:
                robot.moveArm(Robot.Position.MID,0.1);
                break;
            case RIGHT:
                robot.moveArm(Robot.Position.HIGH,0.1);
                break;
        }
        robot.intake(Robot.Direction.OUT,0.5);
    }
}
