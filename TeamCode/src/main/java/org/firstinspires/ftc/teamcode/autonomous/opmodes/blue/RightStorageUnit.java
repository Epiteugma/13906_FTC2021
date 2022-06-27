package org.firstinspires.ftc.teamcode.autonomous.opmodes.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Right Storage Unit", group = "FTC22Auto_Store")
public class RightStorageUnit extends Right {

    @Override
    public void runOpMode() {
        super.runOpMode();
        driveTrain.driveCM(-10, 0.3);
        driveTrain.turn(180, 0.1, 1);
        driveTrain.driveCM(-45, 0.3);
        driveTrain.turn(180, 0.1, 1);
        double strafePower = 0.2;
        driveTrain.runOnEncoders();
        frontRight.resetStallDetection();
        backRight.resetStallDetection();
        frontLeft.resetStallDetection();
        backLeft.resetStallDetection();
        while (!frontRight.isStalled() && !backRight.isStalled() && !frontLeft.isStalled() && !backLeft.isStalled()) {
            // strafe to the left
            frontLeft.setPower(strafePower);
            frontRight.setPower(-strafePower);
            backLeft.setPower(-strafePower);
            backRight.setPower(strafePower);
        }
    }
}