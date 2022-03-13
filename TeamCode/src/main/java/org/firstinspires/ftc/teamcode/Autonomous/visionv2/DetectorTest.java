package org.firstinspires.ftc.teamcode.Autonomous.visionv2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Detector test-training", group="vision")
@Disabled
public class DetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Detector detector = new Detector(hardwareMap);
        telemetry.addData("initializing", "done");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                detector.loadImage();
            }
            // Default position is CENTER so make sure you click 'A' to get the actual position.
            Detector.ElementPosition pos = detector.getElementPosition();
            telemetry.addData("seen objects", pos == Detector.ElementPosition.LEFT ? "left" :
                    (pos == Detector.ElementPosition.RIGHT ? "right" : "Center"));
            telemetry.update();
        }
    }
}
