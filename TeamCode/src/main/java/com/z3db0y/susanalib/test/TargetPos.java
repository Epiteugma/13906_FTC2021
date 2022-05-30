package com.z3db0y.susanalib.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.z3db0y.susanalib.Motor;

@Disabled
@TeleOp(name = "TargetPos Test", group = "SusanaLib")
public class TargetPos extends LinearOpMode {

    public void runOpMode() {
        Motor motor = new Motor(hardwareMap, "duckSpinner2");
        motor.setTargetPosition(0);
        motor.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
        waitForStart();

        while (opModeIsActive()) {
            motor.setTargetPosition(1000);
        }
    }

}
