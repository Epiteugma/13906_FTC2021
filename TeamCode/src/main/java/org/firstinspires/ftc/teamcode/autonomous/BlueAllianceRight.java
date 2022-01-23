package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMototEx;
// Import Location enums. (Detector.Location)
import org.firstinspires.ftc.teamcode.vision.DuckDetector;
import org.firstinspires.ftc.teamcode.vision.TseDetector;

import java.util.Arrays;

@Autonomous(name="Blue Alliance Right", group="FTC22")
public class BlueAllianceRight extends LinearOpMode {
    DcMototEx backLeft;
    DcMototEx backRight;
    DcMototEx frontLeft;
    DcMototEx frontRight;
    DcMototEx armClaw;
    DcMototEx collector;
    DcMototEx duckSpinner1;
    DcMototEx duckSpinner2;
    Robot robot;

    private void initMotors() {
        backLeft = hardwareMap.get(DcMototEx.class, "backLeft");
        frontLeft = hardwareMap.get(DcMototEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMototEx.class, "backRight");
        frontRight = hardwareMap.get(DcMototEx.class, "frontRight");
        armClaw = hardwareMap.get(DcMototEx.class, "armClaw");
        collector = hardwareMap.get(DcMototEx.class, "collector");
        duckSpinner1 = hardwareMap.get(DcMototEx.class, "duckSpinner1");
        duckSpinner2 = hardwareMap.get(DcMototEx.class, "duckSpinner2");
    }
    
    @Override
    public void runOpMode(){
        initMotors();
        robot = new Robot(Arrays.asList(backLeft, frontLeft, backRight, frontRight, armClaw, collector, duckSpinner1, duckSpinner2), this);

        DuckDetector.Location duckPos = robot.getDuckPos();
        waitForStart();

        telemetry.addData("duck", duckPos);
        telemetry.update();
        switch (duckPos) {
            case LEFT:
                break;
            case CENTER:
                robot.drive(Robot.Direction.FORWARDS, 0.5, 5);
                robot.turn(Robot.Direction.LEFT, 0.5, 90);
                break;
            case RIGHT:
                break;
        }
        stop();
    }
}
