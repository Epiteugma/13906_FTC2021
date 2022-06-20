package com.z3db0y.susanalib.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.z3db0y.susanalib.Logger;

@TeleOp(name = "Logger Test", group = "SusanaLib")
public class LooggerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Logger.setTelemetry(telemetry);
        Logger.addData("test");
        Logger.update();
        waitForStart();
        while (opModeIsActive()) {}
    }

}
