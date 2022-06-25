package org.firstinspires.ftc.teamcode.Autonomous.New.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Right Storage Unit", group="FTC22Auto_Store")
public class BlueAllianceRightStorageUnit extends BlueAllianceRight {

    @Override
    public void runOpMode() {
        super.runOpMode();

        driveTrain.turn(180, 0.1, 1);
        driveTrain.driveCM(-50, 0.3);
        driveTrain.turn(180, 0.1, 1);
    }
}