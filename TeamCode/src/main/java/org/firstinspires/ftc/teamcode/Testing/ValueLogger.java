package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Value Logger", group = "FTC22")
public class ValueLogger extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Gamepad", Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)));
            telemetry.update();
        }
    }
}
