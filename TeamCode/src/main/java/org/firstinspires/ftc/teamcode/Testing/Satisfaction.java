package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Satisfaction", group = "Stas insane project")
public class Satisfaction extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            double left1 = 0;
            double right1 = 0;
            double left2 = 0;
            double right2 = 0;
            if(gamepad1.left_trigger > 0) left1 = gamepad1.left_trigger;
            if(gamepad1.right_trigger > 0) right1 = gamepad1.right_trigger;
            gamepad1.rumble(left1, right1, 1000);
            if(gamepad2.left_trigger > 0) left2 = gamepad2.left_trigger;
            if(gamepad2.right_trigger > 0) right2 = gamepad2.right_trigger;
            gamepad2.rumble(left2, right2, 10000);
        }
    }
}
