package com.z3db0y.susanalib.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.z3db0y.susanalib.Motor;

@Disabled
@TeleOp(name = "Run to Position Test", group = "SusanaLib")
public class RunToPosition extends LinearOpMode {
    Motor motor;

    @Override
    public void runOpMode() {
        motor = new Motor(hardwareMap, "duckSpinner2");
        waitForStart();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.runToPosition(2000, 1);
    }

}
