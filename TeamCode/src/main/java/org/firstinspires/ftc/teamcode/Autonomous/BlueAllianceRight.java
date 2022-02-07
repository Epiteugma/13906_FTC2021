package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Autonomous.visionv2.Detector;

import java.util.Arrays;

@Autonomous(name="Blue Alliance Right", group="FTC22")
public class BlueAllianceRight extends LinearOpMode {

    private void initMotors() {
        // Motors, servos, distance sensor and IMU
        BNO055IMU IMU = hardwareMap.get(BNO055IMU.class, "imu");
        SensorRevTOFDistance cargoDetector = new SensorRevTOFDistance(hardwareMap, "cargoDetector");
        Motor duckSpinner1 = new Motor( hardwareMap, "duckSpinner1");
        Motor duckSpinner2 = new Motor( hardwareMap, "duckSpinner2");
        MotorGroup duckSpinners = new MotorGroup(duckSpinner1, duckSpinner2);
        Motor m_arm = new Motor(hardwareMap, "arm");
        Motor m_collector = new Motor(hardwareMap, "collector");
        ServoEx capper= new SimpleServo(hardwareMap, "capper",0,90);
        Motor m_frontRight = new Motor(hardwareMap, "frontRight");
        Motor m_frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor m_backRight = new Motor(hardwareMap, "backRight");
        Motor m_backLeft = new Motor(hardwareMap, "backLeft");

        // grab the internal DcMotor or servo object
        DcMotor frontRight = m_frontRight.motor;
        DcMotor frontLeft = m_frontLeft.motor;
        DcMotor backRight = m_backRight.motor;
        DcMotor backLeft = m_backLeft.motor;
        DcMotor arm = m_arm.motor;
        DcMotor collector = m_collector.motor;
    }
    
    @Override
    public void runOpMode(){
        initMotors();
        Robot robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, arm, collector, duckSpinners), this);

        Detector detector = new Detector(hardwareMap);
        Detector.ElementPosition itemPos = detector.getElementPosition();
        telemetry.addData("Detected Cargo : ", robot.cargoDetection());
        telemetry.update();
        waitForStart();
        switch (itemPos) {
            case Detector.ElementPosition.LEFT:
                robot.drive(Robot.Direction.LEFT, 0.5, 5000);
                break;
            case Detector.ElementPosition.RIGHT:
                robot.strafe(Robot.Direction.LEFT, 0.5, 5000);
                break;
            case Detector.ElementPosition.CENTER:
                robot.turn(Robot.Direction.LEFT, 1, 90);
                break;
        }
    }
}
