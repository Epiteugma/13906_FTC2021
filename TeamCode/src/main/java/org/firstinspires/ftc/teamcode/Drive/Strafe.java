package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Strafe Test", group = "FTC22")
public class Strafe extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        hardwareMap.get(DcMotor.class, "frontLeft").setPower(1);
        hardwareMap.get(DcMotor.class, "frontRight").setPower(1);
        hardwareMap.get(DcMotor.class, "backLeft").setPower(1);
        hardwareMap.get(DcMotor.class, "backRight").setPower(1);
        while (opModeIsActive()) {}
    }

}
