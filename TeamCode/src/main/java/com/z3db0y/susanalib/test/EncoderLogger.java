package com.z3db0y.susanalib.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Logger", group = "SusanaLib")
public class EncoderLogger extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        DcMotor motor = hardwareMap.get(DcMotor.class, "frontLeft");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive()) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Ticks", motor.getCurrentPosition());
            telemetry.update();
        }
    }

}
