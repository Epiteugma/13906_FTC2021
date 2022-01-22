package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// Import Location enums. (Detector.Location)
import org.firstinspires.ftc.teamcode.vision.DuckDetector;
import org.firstinspires.ftc.teamcode.vision.TseDetector;

import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;

@Autonomous(name="Red Alliance Right", group="FTC22")
public class RedAllianceRight extends LinearOpMode {
    OpenCvCamera webcam;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor armClaw;
    DcMotor collector;
    DcMotor duckSpinner1;
    DcMotor duckSpinner2;
    Robot robot;

    private void initMotors() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        armClaw = hardwareMap.get(DcMotor.class, "armClaw");
        collector = hardwareMap.get(DcMotor.class, "collector");
        duckSpinner1 = hardwareMap.get(DcMotor.class, "duckSpinner1");
        duckSpinner2 = hardwareMap.get(DcMotor.class, "duckSpinner2");
    }
    
    @Override
    public void runOpMode(){
        initMotors();
        robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, armClaw, collector, duckSpinner1, duckSpinner2), this);

        DuckDetector.Location duckPos = robot.getDuckPos();
        waitForStart();

        // Move claw down.
        robot.moveClaw(Robot.Position.DOWN);
        // Strafe to the side to stand in front of the shipping hub. -
        // TODO: calibrate time
        robot.strafe(Robot.Direction.LEFT, 0.5, 1000);
        // Move towards the shipping hub.
        robot.drive(Robot.Direction.FORWARDS, 0.5, 1000);
        // Lift the claw to the right level.
        switch(duckPos) {
            case LEFT:
                robot.moveClaw(Robot.Position.HIGH);
                break;
            case RIGHT:
                robot.moveClaw(Robot.Position.LOW);
                break;
            case CENTER:
                robot.moveClaw(Robot.Position.MID);
                break;
        }
        // Release ball/cube onto the level.
        robot.intake(Robot.Direction.OUT);
        // Move back close to wall.
        robot.drive(Robot.Direction.BACKWARDS, 0.5, 1000);
        // Move towards the carousel. -
        // TODO: calibrate time
        robot.strafe(Robot.Direction.LEFT, 0.5, 1000);
        // Spin the carousel.
        robot.duckSpin();
        // Turn 90 degrees towards the parking area.
        // robot.turn(Robot.Direction.RIGHT);
        // Drive the robot into the parking area. -
        // TODO: calibrate time
        robot.drive(Robot.Direction.FORWARDS, 0.5, 1000);
    }
}