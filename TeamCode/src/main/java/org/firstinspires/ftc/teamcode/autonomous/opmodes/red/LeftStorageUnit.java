package org.firstinspires.ftc.teamcode.autonomous.opmodes.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.z3db0y.susanalib.MecanumDriveTrain;

@Autonomous(name = "Red Left Storage Unit", group = "FTC22Auto_Store")
public class LeftStorageUnit extends Left {

    @Override
    public void runOpMode() {
        super.runOpMode();
        driveTrain.driveCM(-10, 0.3);
        driveTrain.turn(180, 0.1, 1);
        driveTrain.driveCM(-45, 0.2);
        driveTrain.turn(180, 0.1, 1);
        driveTrain.strafeCM(MecanumDriveTrain.Side.RIGHT, 20, 0.2);
    }
}