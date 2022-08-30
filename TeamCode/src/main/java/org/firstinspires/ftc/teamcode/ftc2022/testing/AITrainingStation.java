package org.firstinspires.ftc.teamcode.ftc2022.testing;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "AI Training Station", group = "FTC22")
@Disabled
public class AITrainingStation extends LinearOpMode {
    public void runOpMode() {
        waitForStart();
        DcMotor spin = hardwareMap.get(DcMotor.class, "spin");
        while (opModeIsActive()) {
            if(gamepad1.dpad_up) spin.setPower(0.2);
            else if (gamepad1.dpad_down) spin.setPower(-0.2);
            else spin.setPower(0);
        }
    }
}
