package org.firstinspires.ftc.teamcode.Autonomous.New.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Configurable;

@Autonomous(name="Blue Right Warehouse", group="FTC22Auto_Ware")
public class BlueAllianceRightWarehouse extends BlueAllianceRight {

    @Override
    public void runOpMode() {
        super.runOpMode();
        driveTrain.turn(85, 0.1, 1);
        driveTrain.driveCM(100, 0.4);
        arm.runToPositionAsync(Configurable.armHighPosition, 1);
        driveTrain.turn(85, 0.1, 1);
        driveTrain.driveCM(300, 0.4);
    }
}
