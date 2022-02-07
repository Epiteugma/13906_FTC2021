package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Satisfaction", group = "Stas insane project")
public class Satisfaction extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            gamepad1.rumble(1, 1, 1000);
            gamepad2.rumble(1, 1, 1000);
        }
    }
}
