package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Import Location enums. (Detector.Location)
import org.firstinspires.ftc.teamcode.autonomous.vision.DuckDetector;
import org.firstinspires.ftc.teamcode.autonomous.vision.TseDetector;

import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;

@Autonomous(name="Red Alliance Right", group="FTC22")
public class RedAllianceRight extends LinearOpMode {
    OpenCvCamera webcam;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx armClaw;
    DcMotorEx collector;
    DcMotorEx duckSpinner1;
    DcMotorEx duckSpinner2;
    Robot robot;

    private void initMotors() {
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        armClaw = hardwareMap.get(DcMotorEx.class, "armClaw");
        collector = hardwareMap.get(DcMotorEx.class, "collector");
        duckSpinner1 = hardwareMap.get(DcMotorEx.class, "duckSpinner1");
        duckSpinner2 = hardwareMap.get(DcMotorEx.class, "duckSpinner2");
    }
    
    @Override
    public void runOpMode(){
        initMotors();
        robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, armClaw, collector, duckSpinner1, duckSpinner2), this);

        DuckDetector.Location duckPos = robot.getDuckPos();
        waitForStart();

        // Move claw down.
        //robot.moveClaw(Robot.Position.DOWN,1);
        // Strafe to the side to stand in front of the shipping hub. -
        // TODO: calibrate time
        //robot.strafe(Robot.Direction.LEFT, 0.5, 1000);
        // Move towards the shipping hub.
        //robot.drive(Robot.Direction.FORWARDS, 0.5, 1000);
        // Lift the claw to the right level.
        switch(duckPos) {
            case LEFT:
                //robot.moveClaw(Robot.Position.HIGH, 1);
                break;
            case RIGHT:
                robot.drive(Robot.Direction.FORWARDS,1,100);
                break;
            case CENTER:
                robot.drive(Robot.Direction.FORWARDS,1,100);
                //robot.moveClaw(Robot.Position.MID,1);
                break;
        }
        // Release ball/cube onto the level.
//        robot.intake(Robot.Direction.OUT, 1);
//        // Move back close to wall.
//        robot.drive(Robot.Direction.BACKWARDS, 0.5, 1000);
//        // Move towards the carousel. -
//        // TODO: calibrate time
//        robot.strafe(Robot.Direction.LEFT, 0.5, 1000);
//        // Spin the carousel.
//        robot.duckSpin(1000, 1);
//        // Turn 90 degrees towards the parking area.
//        // robot.turn(Robot.Direction.RIGHT);
//        // Drive the robot into the parking area. -
//        // TODO: calibrate time
//        robot.drive(Robot.Direction.FORWARDS, 0.5, 1000);
    }
}